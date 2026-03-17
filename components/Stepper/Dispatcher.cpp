#include "Dispatcher.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

void Dispatcher::init(const StepperConfig& config)
{
    stepper.init(config);
    reset();
}

void Dispatcher::reset() {
    stepper.reset();
}

void Dispatcher::dispatch(const BasePlannerBlock* block) {
    auto profilePtr = block->createSpeedProfile(profileSlot.storage);
    if(profilePtr){
        stepper.move(profilePtr, 0);
        profilePtr->~Vec3SpeedProfile();
    }
}

void Dispatcher::dispatchCallback(FixedFunction<void()>& callback) {
    if(callback){
        stepper.addCallback(callback, 0);
    }
}

void Dispatcher::waitQueueAvailable() const
{
    while(!stepper.queueAvailable()){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool Dispatcher::needMore() const
{
    if(stepper.queueAvailable() && stepper.getRemainingQueueTime() < timeHorizon){
        return true;
    }
    return false;
}

bool Dispatcher::needMore(float overrideTimeHorizon) const
{
    if(stepper.queueAvailable() && stepper.getRemainingQueueTime() < overrideTimeHorizon){
        return true;
    }
    return false;
}
