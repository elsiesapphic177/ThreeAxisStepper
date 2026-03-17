#pragma once
#define EnablePulseDutySignal false
#define EnableStepperDutySignal false
#define EnablePlannerDutySignal false
#define GPIO_OUTPUT_LOCK false // set to true to disable pin output but keep all other functions, for testing

// Pulse generator configuration
#define PULSE_QUEUE_LENGTH 16 // Must be a power of 2 for FastISRQueue, and should be large enough to hold pending pulses without overflow.
static const uint32_t StepPulseWidth = 20;          // us
static const uint32_t PulsePrepareTime = 50;         // us, time for force-enable/disable decision
static const uint32_t PulseAlarmDelay = 1000000;  // us (1s), alarm when pulse queue full

// Stepper configuration
#define NewtonStepperAvailable false
#define AccumulatedStepInit 0.5f
#define StepLostLimit 2 // mismatch between target position and pulse count. 
#define MaxVelocityOffsetSteps 100 // for pid control, max velocity offset is 100 steps/s
#define MaxIntegralErrorSteps 50  // anti-windup: max integral error in steps
#define MotionQueueSize 8 
#define DeltaTime 0.001f // time for trimming the profile, too large might cause pulse gen queue full
#define CallbackQueueSize 8
#define DISABLE_MOTOR_WAIT_TIME 1000000 // us to wait before disabling motor after motion complete, to ensure all pulses are sent

// Planner
#define MIN_SEGMENT_TIME 0.005f  // seconds; segments shorter than this are merged (0 = disabled)
#define PLANNER_BLOCK_BUFFER_SIZE 100
#define JOG_SEGMENT_TIME 0.05f  // seconds per cruise segment
#define PLANNER_TASK_STACK   4096
#define PLANNER_TASK_PRIO    2
#define QUEUE_POLL_MS        10     // fast poll: queue receive, state checks (ms)
#define IDLE_POLL_MS         50     // slow poll: stop / resume / idle waits (ms)

#if (EnableTaskDutySignal || EnablePulseDutySignal || EnableStepperDutySignal)
#include "soc/gpio_struct.h"
inline void gpio_set_level_fast(gpio_num_t pin, bool level){
    if(level) GPIO.out_w1ts = (1ULL << pin);
    else      GPIO.out_w1tc = (1ULL << pin);
}
inline bool gpio_check_level_set(gpio_num_t pin){
    return (GPIO.out & (1ULL << pin)) != 0;
}
#define DutySignalPin GPIO_NUM_26
#define DutySignalConfig() static bool hasDutySignal = false;
#define DutySignalStart() hasDutySignal = gpio_check_level_set(DutySignalPin);
#define DutySignalOn() if(!hasDutySignal) gpio_set_level_fast(DutySignalPin, 1);
#define DutySignalOff() if(!hasDutySignal) gpio_set_level_fast(DutySignalPin, 0);
#endif

#if EnablePulseDutySignal
#define PulseDutySignalConfig() DutySignalConfig()
#define PulseDutySignalStart() DutySignalStart()
#define PulseDutySignalOn() DutySignalOn()
#define PulseDutySignalOff() DutySignalOff()
#else
#define PulseDutySignalConfig()
#define PulseDutySignalStart()
#define PulseDutySignalOn()
#define PulseDutySignalOff()
#endif

#if EnableStepperDutySignal
#define StepperDutySignalConfig() DutySignalConfig()
#define StepperDutySignalStart() DutySignalStart()
#define StepperDutySignalOn() DutySignalOn()
#define StepperDutySignalOff() DutySignalOff()
#else
#define StepperDutySignalConfig()
#define StepperDutySignalStart()
#define StepperDutySignalOn()
#define StepperDutySignalOff()
#endif

#if EnablePlannerDutySignal
#define PlannerDutySignalConfig() DutySignalConfig()
#define PlannerDutySignalStart() DutySignalStart()
#define PlannerDutySignalOn() DutySignalOn()
#define PlannerDutySignalOff() DutySignalOff()
#else
#define PlannerDutySignalConfig()
#define PlannerDutySignalStart()
#define PlannerDutySignalOn()
#define PlannerDutySignalOff()
#endif