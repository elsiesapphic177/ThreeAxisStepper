#include "PulseGenerator.h"
#include "MemoryUtil.h"
#include <driver/mcpwm_prelude.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_attr.h>
#include <esp_log.h>
#include "esp_timer.h"
#include "hal/mcpwm_ll.h"

const char* MCPWMGenTAG = "MCPWMPulseGenerator";
struct MCPWMPulseGenerator::Impl{
    static int totalId;
    mcpwm_timer_handle_t waveformTimer{NULL};
    mcpwm_gen_handle_t stepGen{NULL};
    mcpwm_gen_handle_t dirGen{NULL};
    mcpwm_sync_handle_t soft_sync_source{NULL};
    mcpwm_dev_t* dev{nullptr}; // cached MCPWM device for direct LL access in ISR
    int groupId;
    int timerId;
    int operId; // operator ID (= timerId, one operator per timer allocated in order)
    bool hasInit{false};

    uint32_t excessTicks{0};
    bool nextPulseDirFlag{false};
    bool hasForce{false};
    bool dirFlag{false}; // false for negative direction, true for positive direction
    bool dirInverse{false}; // invert the physical direction pin output
    FastISRQueue<uint32_t, PULSE_QUEUE_LENGTH> pulseQueue;
    volatile bool runFlag{false};
    volatile bool stop_requested{false};
    volatile bool skipPulse{false}; // true on first ISR after start: skip setupPulse, go straight to loadNextPulseData
    volatile bool idle_requested{false}; // set when queue empty in loadNextPulseData; next ISR will stop if still empty
    static void IRAM_ATTR setPeriod(Impl* gen, uint32_t period);
    static bool IRAM_ATTR setupPulse(Impl* gen);
    static void IRAM_ATTR loadNextPulseData(Impl* gen);
    static bool IRAM_ATTR timerCallback(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx);

    void init(gpio_num_t targetPin_in, gpio_num_t dirPin_in, bool dirInverse_in);
    void resetTimer();  

    esp_timer_handle_t queueFullAlarm, pulseRejectAlarm;
    static void queueFullAlarmCallback(void* arg);
    static void pulseRejectAlarmCallback(void* arg);
    
    int32_t count{0};
};
int MCPWMPulseGenerator::Impl::totalId = 0;

void IRAM_ATTR MCPWMPulseGenerator::Impl::setPeriod(Impl *gen, uint32_t period_in)
{
    if(__builtin_expect(period_in == 0, 0)){
        esp_rom_printf("period is 0");
    }
    if(period_in < MCPWM_LL_MAX_COUNT_VALUE){
        mcpwm_ll_timer_set_peak(gen->dev, gen->timerId, period_in, false);
        gen->excessTicks = 0;
    }
    else {
        uint32_t period = MCPWM_LL_MAX_COUNT_VALUE-1;
        if(period > period_in - (PulsePrepareTime<<1))
            period = period_in - (PulsePrepareTime<<1);
        mcpwm_ll_timer_set_peak(gen->dev, gen->timerId, period, false);
        gen->excessTicks = period_in - period;
    }
}

bool IRAM_ATTR MCPWMPulseGenerator::Impl::setupPulse(Impl *gen)
{
    if(__builtin_expect(gen->excessTicks == 0, 1)){
        if(gen->hasForce){
#if !GPIO_OUTPUT_LOCK
            mcpwm_ll_gen_disable_continue_force_action(gen->dev, gen->operId, 0); // release step gen
            #endif
            gen->hasForce = false;
        }
        if(gen->nextPulseDirFlag != gen->dirFlag){
            gen->dirFlag = gen->nextPulseDirFlag;
#if !GPIO_OUTPUT_LOCK
            mcpwm_ll_gen_set_continue_force_level(gen->dev, gen->operId, 1, (gen->dirFlag ^ gen->dirInverse)? 1 : 0); // dir gen
#endif
        }
        gen->count += gen->dirFlag ? 1 : -1;
        return true;
    }
    else{
        if(!gen->hasForce){
#if !GPIO_OUTPUT_LOCK
            mcpwm_ll_gen_set_continue_force_level(gen->dev, gen->operId, 0, 0); // step gen low
#endif
            gen->hasForce = true;
        }
        setPeriod(gen, gen->excessTicks);
        return false;
    }
}

void IRAM_ATTR MCPWMPulseGenerator::Impl::loadNextPulseData(Impl *gen)
{
    uint32_t countValue = mcpwm_ll_timer_get_count_value(gen->dev, gen->timerId);
    uint32_t now = static_cast<uint32_t>(esp_timer_get_time()) + PulsePrepareTime - countValue;
    uint32_t encoded;
    if(!gen->pulseQueue.pop(encoded))
    {
        // Don't stop immediately — the current pulse is still being output.
        // Defer stop to next ISR so addPulse won't clobber the in-flight pulse.
        setPeriod(gen, PulsePrepareTime + StepPulseWidth + 2);
        gen->idle_requested = true;
        return;
    }
    uint32_t dt = (encoded & ~1U) - now;
    if(dt > PulsePrepareTime + StepPulseWidth + 1 && dt < 0x0FFFFFFFUL){ // next pulse time is not passed
        setPeriod(gen, dt);
        gen->nextPulseDirFlag = encoded & 1U;
        return;
    }
    else{// set period to minimum try to catch up
        setPeriod(gen, PulsePrepareTime + StepPulseWidth + 2);
        gen->nextPulseDirFlag = encoded & 1U;
    }
}

PulseDutySignalConfig();
bool IRAM_ATTR MCPWMPulseGenerator::Impl::timerCallback(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    PulseDutySignalStart();
    PulseDutySignalOn();
    Impl* gen = (Impl*)user_ctx;
    if(__builtin_expect(gen->stop_requested, 0)){
        mcpwm_ll_timer_set_peak(gen->dev, gen->timerId, PulsePrepareTime >> 1, false);
        mcpwm_ll_timer_set_start_stop_command(gen->dev, gen->timerId, MCPWM_TIMER_STOP_FULL);
        if(!gen->hasForce){
#if !GPIO_OUTPUT_LOCK
            mcpwm_ll_gen_set_continue_force_level(gen->dev, gen->operId, 0, 0); // step gen low
#endif
            gen->hasForce = true;
        }
        gen->runFlag = false;
        gen->stop_requested = false;
        gen->idle_requested = false;
    }
    else if(__builtin_expect(gen->idle_requested, 0)){
        // Previous ISR found queue empty. Force LOW (pulse already completed),
        // then check queue again — addPulse may have pushed new data since.
        gen->idle_requested = false;
        if(!gen->hasForce){
#if !GPIO_OUTPUT_LOCK
            mcpwm_ll_gen_set_continue_force_level(gen->dev, gen->operId, 0, 0); // step gen low
#endif
            gen->hasForce = true;
        }
        if(!gen->pulseQueue.isEmpty()){
            // New data arrived, keep running — load it for the next cycle
            loadNextPulseData(gen);
        }
        else{
            mcpwm_ll_timer_set_peak(gen->dev, gen->timerId, PulsePrepareTime >> 1, false);
            mcpwm_ll_timer_set_start_stop_command(gen->dev, gen->timerId, MCPWM_TIMER_STOP_FULL);
            gen->runFlag = false;
        }
    }
    else if(__builtin_expect(gen->skipPulse, 0)){
        // First ISR after start: skip pulse output, just load first pulse from queue
        gen->skipPulse = false;
        loadNextPulseData(gen);
    }
    else {
        if(setupPulse(gen))
            loadNextPulseData(gen);
    }
    PulseDutySignalOff();
    return false;
}

void MCPWMPulseGenerator::Impl::init(gpio_num_t targetPin_in, gpio_num_t dirPin_in, bool dirInverse_in)
{
    if(totalId >= 6){
        ESP_LOGE(MCPWMGenTAG,"No more timer resource for new pulse generator");
        return;
    }
    if(hasInit) return;
    hasInit = true;
    dirInverse = dirInverse_in;
    groupId = totalId / 3;
    timerId = totalId % 3;
    operId = timerId; // one operator per timer, allocated in order
    dev = groupId == 0 ? &MCPWM0 : &MCPWM1;
    totalId++;
    // single waveform timer init
    mcpwm_timer_config_t pwmTimer_config = {
        .group_id = groupId,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, 
        .resolution_hz = 1000000,           
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,    
        .period_ticks = MCPWM_LL_MAX_COUNT_VALUE-1,
        .intr_priority = 2,
        .flags = {
            .update_period_on_empty = false,
            .update_period_on_sync = false,
        }
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&pwmTimer_config, &waveformTimer));
    
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = { .group_id = groupId };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, waveformTimer));

    mcpwm_generator_config_t gen_config = { .gen_gpio_num = targetPin_in };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &stepGen));
    mcpwm_generator_config_t dir_gen_config = { .gen_gpio_num = dirPin_in };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &dir_gen_config, &dirGen));

#if !GPIO_OUTPUT_LOCK
    mcpwm_cmpr_handle_t comparatorA = NULL;
    mcpwm_comparator_config_t cmpr_config = { .flags = {0} }; 
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &comparatorA));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, PulsePrepareTime));
    mcpwm_cmpr_handle_t comparatorB = NULL;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_config, &comparatorB));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, PulsePrepareTime + StepPulseWidth));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(stepGen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorA, MCPWM_GEN_ACTION_HIGH))); 
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(stepGen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorB, MCPWM_GEN_ACTION_LOW))); 
#endif

    mcpwm_soft_sync_config_t soft_sync_config = {};
    ESP_ERROR_CHECK(mcpwm_new_soft_sync_src(&soft_sync_config, &soft_sync_source));
    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .sync_src = soft_sync_source,  
        .count_value = 0,              
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(waveformTimer, &sync_phase_config));

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = NULL,
        .on_empty = Impl::timerCallback,
        .on_stop = NULL,
    };
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(waveformTimer, &cbs, this));
    ESP_ERROR_CHECK(mcpwm_timer_enable(waveformTimer));

    // alarm for debugging pulse timing
    esp_timer_create_args_t timer_config = {
        .callback = Impl::queueFullAlarmCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PulseQueueFullAlarm",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config,&queueFullAlarm));
    timer_config.callback = Impl::pulseRejectAlarmCallback;
    timer_config.name = "PulseRejectAlarm";
    ESP_ERROR_CHECK(esp_timer_create(&timer_config,&pulseRejectAlarm));
}

void MCPWMPulseGenerator::Impl::resetTimer()
{
    if(runFlag){
        stop_requested = true;
        idle_requested = false;
        mcpwm_soft_sync_activate(soft_sync_source);
    }
    excessTicks = 0;
    pulseQueue.clear();
}

void MCPWMPulseGenerator::Impl::queueFullAlarmCallback(void *arg)
{
    ESP_LOGE(MCPWMGenTAG, "Pulse queue full, alarm triggered");
}

void MCPWMPulseGenerator::Impl::pulseRejectAlarmCallback(void *arg)
{
    ESP_LOGW(MCPWMGenTAG, "Pulse rejected due to small interval, alarm triggered");
}

MCPWMPulseGenerator::~MCPWMPulseGenerator()
{
    if(impl)
        delete impl;
    impl = nullptr;
}

void MCPWMPulseGenerator::init(gpio_num_t targetPin_in, gpio_num_t dirPin_in, bool dirInverse)
{
    if(!impl)
        impl = new Impl();
    impl->init(targetPin_in, dirPin_in, dirInverse);
}

void MCPWMPulseGenerator::addPulse(const int64_t& pulseTime, bool direction)
{
    // Bit 0 = direction, bits 31:1 = time (±1μs precision, negligible vs 20μs minimum)
    uint32_t encoded = (static_cast<uint32_t>(pulseTime) & ~1U) | static_cast<uint32_t>(direction);
    if(!impl->pulseQueue.push(encoded)){
        if(!esp_timer_is_active(impl->queueFullAlarm)){
            esp_timer_start_once(impl->queueFullAlarm, PulseAlarmDelay);
        }
        return;
    }
    if(!impl->runFlag){
        // Force step low, mark first ISR to skip pulse output and just load from queue
#if !GPIO_OUTPUT_LOCK
        mcpwm_ll_gen_set_continue_force_level(impl->dev, impl->operId, 0, 0);
        // Sync dir pin with dirFlag — after init or reset the physical pin may not match dirFlag
        mcpwm_ll_gen_set_continue_force_level(impl->dev, impl->operId, 1, (impl->dirFlag ^ impl->dirInverse) ? 1 : 0);
#endif
        impl->hasForce = true;
        impl->skipPulse = true;        
        // Set a short period so the first ISR fires quickly after start
        mcpwm_ll_timer_set_peak(impl->dev, impl->timerId, PulsePrepareTime, false);
        mcpwm_soft_sync_activate(impl->soft_sync_source); // reset counter to 0
        impl->runFlag = true;
        __asm__ __volatile__ ("memw" ::: "memory");
        mcpwm_timer_start_stop(impl->waveformTimer, MCPWM_TIMER_START_NO_STOP);
    }
}

void MCPWMPulseGenerator::reset()
{
    if(impl)
        impl->resetTimer();
}

bool MCPWMPulseGenerator::isRunning() const
{
    if(impl)
        return impl->runFlag;
    return false;
}

int32_t MCPWMPulseGenerator::getCount() const
{
    if(impl)
        return impl->count;
    return 0;
}

void MCPWMPulseGenerator::setCount(int32_t count)
{
    if(impl)
        impl->count = count;
}
