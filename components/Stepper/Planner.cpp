#include "Planner.h"
#include <cmath>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

PlannerDutySignalConfig();
enum PlannerNotify: uint32_t{
    NOTIFY_START = 1,
    NOTIFY_RESET = 2,
    NOTIFY_JOG   = 3,
    NOTIFY_STOP  = 4,
    NOTIFY_RESUME = 5,
};

void Planner::setState(PlannerState newState)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state = newState;
    xSemaphoreGive(stateMutex);
}

Planner::PlannerState Planner::getState()
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    PlannerState currentState = state;
    xSemaphoreGive(stateMutex);
    return currentState;
}

void Planner::checkNotify(TickType_t timeout)
{
    uint32_t notifyFlag;
    PlannerDutySignalOff();
    if(xTaskNotifyWait(0,ULONG_MAX,&notifyFlag, timeout) == pdTRUE){ // high priority
        PlannerDutySignalOn();
        if(notifyFlag == NOTIFY_START){
            if(state == IDLE)
                setState(Planner::NORMAL);
        }
        else if(notifyFlag == NOTIFY_RESET){
            setState(Planner::IDLE);
        }
        else if(notifyFlag == NOTIFY_JOG){
            if(state == IDLE){
                jogAccelSegment();
                setState(Planner::JOGGING);
            }
        }
        else if(notifyFlag == NOTIFY_STOP){
            if(state == Planner::JOGGING)
                setState(Planner::JOG_DECEL);
            else if(state == Planner::NORMAL || state == Planner::FLUSHING)
                setState(Planner::STOPPING);
        }
        else if(notifyFlag == NOTIFY_RESUME){
            if(state == Planner::STOPPED)
                setState(preStopState);
        }
    }
}

void Planner::plannerTask(void *arg)
{
    Planner* planner = static_cast<Planner*>(arg);
    PlannerDutySignalStart();
    PlannerDutySignalOn();
    uint32_t notifyFlag;
    while(true) {
        planner->checkNotify(0);
        if(planner->state == Planner::IDLE){
            planner->speedPlanner.setEndPosition(planner->getCurrentPos());// ensure speed planner's end position is correct when idle, so next moveTo() can compute correct distance and speed
            planner->checkNotify(portMAX_DELAY);
        }
        else if(planner->state == Planner::NORMAL){
            if(planner->dispatcher.needMore()){
                Command cmd;
                PlannerDutySignalOff();
                BaseType_t result = xQueueReceive(planner->moveQueue, &cmd, pdMS_TO_TICKS(QUEUE_POLL_MS));
                PlannerDutySignalOn();
                if(result == pdTRUE){
                    if(cmd.type == Command::Flush){
                        if(planner->speedPlanner.hasPendingMerge() &&
                           planner->speedPlanner.count() >= planner->speedPlanner.capacity()){
                            planner->speedPlanner.recalculate();
                            planner->speedPlanner.dispatchFront(planner->dispatcher);
                        }
                        planner->speedPlanner.flushPendingMerge(); // commit any buffered short segment
                        planner->setState(Planner::FLUSHING);
                    }
                    else if(cmd.type == Command::Callback){
                        if(planner->speedPlanner.hasPendingMerge() &&
                           planner->speedPlanner.count() >= planner->speedPlanner.capacity()){
                            planner->speedPlanner.recalculate();
                            planner->speedPlanner.dispatchFront(planner->dispatcher);
                        }
                        planner->speedPlanner.flushPendingMerge(); // callback needs a position; commit pending
                        auto cb = planner->callbackBuffer.acquire();
                        planner->addCallbackBlock(*cb);
                    }
                    else{
                        planner->addBlock(cmd);
                    }
                }
                else{
                    if(planner->speedPlanner.hasPendingMerge()){
                        // Dispatcher needs more and no command arrived —
                        // flush the pending short segment to keep the
                        // stepper fed (this is the "last piece" case).
                        planner->speedPlanner.flushPendingMerge();
                    }
                    else if(planner->speedPlanner.count() == 0 && planner->dispatcher.stepperComplete()){// no more blocks and queue to run, stepper idle
                        if(uxQueueMessagesWaiting(planner->moveQueue) == 0){
                            planner->setState(Planner::IDLE);
                        }
                    }
                }
            }
            else{
                // Stepper queue full — back off before re-checking.
                planner->checkNotify(pdMS_TO_TICKS(QUEUE_POLL_MS));
            }
        }
        else if(planner->state == Planner::FLUSHING){
            if(planner->dispatcher.needMore()){
                planner->flushBlock();
            }
            else{
                planner->checkNotify(pdMS_TO_TICKS(QUEUE_POLL_MS));
            }
        }
        else if(planner->state == Planner::STOPPING){
            if(planner->speedPlanner.hasPendingMerge()){
                if(planner->speedPlanner.count() >= planner->speedPlanner.capacity()){
                    planner->speedPlanner.recalculate();
                    planner->speedPlanner.dispatchFront(planner->dispatcher);
                }
                planner->speedPlanner.flushPendingMerge();
            }
            float stoppingDistance = planner->speedPlanner.calcStoppingDistance();
            int blocksToStop = 0;
            for(int i=0;i<planner->speedPlanner.count();i++){
                float dis = planner->speedPlanner.distanceAtBlock(i);
                blocksToStop++;
                stoppingDistance -= dis;
                if(stoppingDistance <= 0)
                    break;
            }
            planner->speedPlanner.recalculateBlocks(blocksToStop);
            while(blocksToStop > 0){
                PlannerDutySignalOff();
                planner->dispatcher.waitQueueAvailable();
                PlannerDutySignalOn();
                planner->speedPlanner.dispatchFront(planner->dispatcher);
                blocksToStop--;
            }
            while(!planner->dispatcher.stepperComplete()){
                PlannerDutySignalOff();
                vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
                PlannerDutySignalOn();
            }
            planner->setState(Planner::STOPPED);
        }
        else if(planner->state == Planner::STOPPED){
            planner->checkNotify(portMAX_DELAY);
        }
        else if(planner->state == Planner::JOGGING){
            // Read jogLastCallTime under mutex to prevent torn 64-bit reads
            xSemaphoreTake(planner->stateMutex, portMAX_DELAY);
            int64_t lastCall = planner->jogLastCallTime;
            xSemaphoreGive(planner->stateMutex);
            if(esp_timer_get_time() > lastCall){
                planner->setState(Planner::JOG_DECEL);
            }
            else if(planner->dispatcher.needMore(JOG_SEGMENT_TIME)){
                planner->jogCruiseSegment();
            }
            else{
                planner->checkNotify(pdMS_TO_TICKS(QUEUE_POLL_MS));
            }
        }
        else if(planner->state == Planner::JOG_DECEL){
            planner->jogDecelSegment();
            // Wait for stepper to complete all segments
            while(!planner->dispatcher.stepperComplete()){
                PlannerDutySignalOff();
                vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
                PlannerDutySignalOn();
            }
            planner->setState(Planner::IDLE);
        }
    }
}

void Planner::addBlock(Command &cmd)
{
    // If speed planner is full, force-dispatch one block (blocking)
    if(speedPlanner.count() >= speedPlanner.capacity()) {
        speedPlanner.recalculate();
        speedPlanner.dispatchFront(dispatcher);
    }

    // Add block
    if(cmd.type == Command::Move){
        speedPlanner.addBlock(cmd.position, cmd.speed);
    }
    else if(cmd.type == Command::Wait){
        // flushPendingMerge + addWaitBlock each need a slot; ensure room.
        if(speedPlanner.hasPendingMerge() &&
           speedPlanner.count() >= speedPlanner.capacity()) {
            speedPlanner.recalculate();
            speedPlanner.dispatchFront(dispatcher);
        }
        speedPlanner.flushPendingMerge(); // wait requires full stop; commit pending
        speedPlanner.addWaitBlock(cmd.speed);
    }
}

void Planner::addCallbackBlock(FixedFunction<void()>& cb)
{
    if(speedPlanner.count() >= speedPlanner.capacity()) {
        speedPlanner.recalculate();
        speedPlanner.dispatchFront(dispatcher);
    }
    speedPlanner.addCallback(cb);
}

void Planner::flushBlock()
{
    if(speedPlanner.count() > 0){
        speedPlanner.recalculate();
        speedPlanner.dispatchFront(dispatcher);
    }
    else if(speedPlanner.hasPendingMerge()){
        // Last short segment — push it so FLUSHING can dispatch it.
        speedPlanner.flushPendingMerge();
    }
    else{ // finish flushing back to normal state
        setState(Planner::NORMAL);
    }
}

void Planner::init(const StepperConfig &config)
{
#ifdef DutySignalPin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    io_conf.pin_bit_mask = (1ULL << DutySignalPin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(DutySignalPin, 0));
#endif

    speedPlanner.init();
    dispatcher.init(config);

    moveQueue = xQueueCreate(PLANNER_BLOCK_BUFFER_SIZE, sizeof(Command));
    stateMutex = xSemaphoreCreateMutex();

    xTaskCreate(plannerTask, "PlannerTask", PLANNER_TASK_STACK, this, PLANNER_TASK_PRIO, &plannerTaskHandle);
}

void Planner::reset()
{
    if(getState() != Planner::IDLE){
        xTaskNotify(plannerTaskHandle, NOTIFY_RESET, eSetValueWithOverwrite);
    }
    while(true){ // wait for planner task to acknowledge reset → IDLE
        if(getState() == Planner::IDLE)
            break;
        vTaskDelay(pdMS_TO_TICKS(QUEUE_POLL_MS));
    }
    dispatcher.reset();
    speedPlanner.reset();
    callbackBuffer.clear();
    xQueueReset(moveQueue);
}

bool Planner::moveTo(const Vec3 &position, float speed, TickType_t timeout)
{
    PlannerState currentState = getState();
    if(currentState != NORMAL && currentState != FLUSHING && currentState != IDLE)
        return false;

    Command cmd{Command::Move, position, speed};
    if(xQueueSend(moveQueue, &cmd, timeout) != pdTRUE)
        return false;
    if(currentState == IDLE){
        xTaskNotify(plannerTaskHandle, NOTIFY_START, eSetValueWithOverwrite);
    }
    return true;
}

bool Planner::addWait(float time_s, TickType_t timeout)
{    
    PlannerState currentState = getState();
    if(currentState != NORMAL && currentState != FLUSHING && currentState != IDLE)
        return false;

    Command cmd{Command::Wait, {}, time_s};
    if(xQueueSend(moveQueue, &cmd, timeout) != pdTRUE)
        return false;
    if(currentState == IDLE){
        xTaskNotify(plannerTaskHandle, NOTIFY_START, eSetValueWithOverwrite);
    }
    return true;
}

bool Planner::addCallback(FixedFunction<void()> cb, TickType_t timeout)
{
    PlannerState currentState = getState();
    if(currentState != NORMAL && currentState != FLUSHING && currentState != IDLE)
        return false;
    if(!callbackBuffer.push(std::move(cb), timeout))
        return false;
    Command cmd{Command::Callback, {}, {}};
    if(xQueueSend(moveQueue, &cmd, timeout) != pdTRUE){
        callbackBuffer.pop_back(); // rollback callback buffer if queue send fails
        return false;
    }
    if(currentState == IDLE){
        xTaskNotify(plannerTaskHandle, NOTIFY_START, eSetValueWithOverwrite);
    }
    return true;
}

bool Planner::flush(bool waitForCompletion, TickType_t timeout)
{
    PlannerState currentState = getState();
    if(currentState != NORMAL && currentState != FLUSHING && currentState != IDLE)
        return false;
    Command cmd{Command::Flush, {}, {}};
    if(xQueueSend(moveQueue, &cmd, timeout) != pdTRUE)
        return false;
    if(currentState == IDLE){
        xTaskNotify(plannerTaskHandle, NOTIFY_START, eSetValueWithOverwrite);
    }
    if(waitForCompletion)
        waitForIdle();
    return true;
}

void Planner::waitForIdle()
{   
    while(true){
        PlannerState currentState = getState();
        if(xTaskNotifyWait(0, 0, nullptr, 0) == pdTRUE || currentState == IDLE || currentState == STOPPED)
            break;
        vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
    }
}

bool Planner::isFinish()
{   
    if(dispatcher.getStepper().isIdle() && uxQueueMessagesWaiting(moveQueue) == 0 && speedPlanner.count() == 0){
        PlannerState currentState = getState();
        if(currentState == IDLE || currentState == STOPPED){
                return true;
        }
    }
    return false;
}

void Planner::stop()
{
    PlannerState currentState = getState();
    bool canStop = false;
    if(currentState == Planner::NORMAL || currentState == Planner::FLUSHING){
        canStop = true;
        preStopState = currentState;
    }
    if(canStop || currentState == Planner::JOGGING){
        xTaskNotify(plannerTaskHandle, NOTIFY_STOP, eSetValueWithOverwrite);
    }
}

void Planner::resume()
{
    PlannerState currentState = getState();
    if(currentState != Planner::STOPPED && currentState != Planner::STOPPING)
        return;
    if(currentState == Planner::STOPPING){
        while(true){
            if(getState() == Planner::STOPPED)
                break;
            vTaskDelay(pdMS_TO_TICKS(IDLE_POLL_MS));
        }
    }
    xTaskNotify(plannerTaskHandle, NOTIFY_RESUME, eSetValueWithOverwrite);
}

void Planner::emergency_stop()
{
    dispatcher.getStepper().emergencyStop();
    reset();
    dispatcher.getStepper().emergencyStopRelease();
}

void Planner::setCurrentPos(const Vec3 &pos)
{
    dispatcher.getStepper().setCurrentPosition(pos);
    speedPlanner.setEndPosition(pos);
}

Vec3 Planner::getCurrentPos()
{
    return dispatcher.getStepper().getCurrentPosition();
}

// --- Jog implementation ---

Vec3 Planner::jogDirection() const
{
    switch(jogAxis){
        case JOG_X_POS: return Vec3{ 1, 0, 0};
        case JOG_X_NEG: return Vec3{-1, 0, 0};
        case JOG_Y_POS: return Vec3{ 0, 1, 0};
        case JOG_Y_NEG: return Vec3{ 0,-1, 0};
        case JOG_Z_POS: return Vec3{ 0, 0, 1};
        case JOG_Z_NEG: return Vec3{ 0, 0,-1};
        default:        return Vec3{ 0, 0, 0};
    }
}

void Planner::jogAccelSegment()
{
    // Dispatch a 0 → jogSpeed ramp along the jog axis
    float maxAccel = speedPlanner.getMaxAcceleration();
    float maxJerk  = speedPlanner.getMaxJerk();
    float speed    = std::min(jogSpeed, speedPlanner.getMaxSpeed());
    Vec3  dir      = jogDirection();

    // Compute how much distance the accel ramp needs
    float tjA, taConst;
    computeAccelTiming(speed, maxAccel, maxJerk, tjA, taConst);
    float accelDist = computeRampDist(0, tjA, taConst, maxJerk);
    if(accelDist < 1e-7f) accelDist = 1e-7f;

    SCurveSegmentProfile profile;
    profile.setup(dir, 0, speed, speed, maxAccel, maxJerk, accelDist);

    dispatcher.getStepper().move(&profile, 0);
    
    speedPlanner.setEndPosition(speedPlanner.getEndPosition() + dir * accelDist);
}

void Planner::jogCruiseSegment()
{
    // Dispatch a constant-speed segment (JOG_SEGMENT_TIME duration)
    float maxAccel = speedPlanner.getMaxAcceleration();
    float maxJerk  = speedPlanner.getMaxJerk();
    float speed    = std::min(jogSpeed, speedPlanner.getMaxSpeed());
    Vec3  dir      = jogDirection();
    float dist     = speed * JOG_SEGMENT_TIME;
    if(dist < 1e-7f) return;

    SCurveSegmentProfile profile;
    profile.setup(dir, speed, speed, speed, maxAccel, maxJerk, dist);

    dispatcher.getStepper().move(&profile, 0);
    
    speedPlanner.setEndPosition(speedPlanner.getEndPosition() + dir * dist);
}

void Planner::jogDecelSegment()
{
    // Dispatch a jogSpeed → 0 ramp, then wait for stepper to finish
    float maxAccel = speedPlanner.getMaxAcceleration();
    float maxJerk  = speedPlanner.getMaxJerk();
    float speed    = std::min(jogSpeed, speedPlanner.getMaxSpeed());
    Vec3  dir      = jogDirection();

    float tjD, tdConst;
    computeAccelTiming(speed, maxAccel, maxJerk, tjD, tdConst);
    float decelDist = computeRampDist(0, tjD, tdConst, maxJerk);
    if(decelDist < 1e-7f) decelDist = 1e-7f;

    // Wait for queue space
    while(!dispatcher.getStepper().queueAvailable()){
        vTaskDelay(pdMS_TO_TICKS(QUEUE_POLL_MS));
    }

    SCurveSegmentProfile profile;
    profile.setup(dir, speed, speed, 0, maxAccel, maxJerk, decelDist);
    dispatcher.getStepper().move(&profile, 0);
    
    speedPlanner.setEndPosition(speedPlanner.getEndPosition() + dir * decelDist);
}

bool Planner::jog(int keepTimeUs)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if(state == JOGGING){
        // Refresh timeout — keep jogging (written under lock to prevent torn 64-bit access)
        jogLastCallTime = esp_timer_get_time() + keepTimeUs;
        xSemaphoreGive(stateMutex);
        return true;
    }
    if(state != IDLE){
        // Can only start jog from IDLE (JOG_DECEL, NORMAL, etc. → reject)
        xSemaphoreGive(stateMutex);
        return false;
    }
    // Start jog from IDLE — set state under lock to eliminate TOCTOU with moveTo/notify
    jogLastCallTime = esp_timer_get_time() + keepTimeUs;
    xSemaphoreGive(stateMutex);
    xTaskNotify(plannerTaskHandle, NOTIFY_JOG, eSetValueWithOverwrite);
    return true;
}
