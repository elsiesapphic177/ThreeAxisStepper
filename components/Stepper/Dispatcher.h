#pragma once
#include "Stepper.h"

// A single linear motion block used throughout the planning pipeline.
class BasePlannerBlock{
public:
    virtual ~BasePlannerBlock() = default;
    virtual Vec3SpeedProfile* createSpeedProfile(void* memory) const = 0;
};

// Converts planned blocks into motion profiles and pushes them to the stepper,
class Dispatcher {
private:
    ThreeAxisStepper  stepper;    
    struct ProfileSlot {
        alignas(8) uint8_t storage[MaxProfileSize];
    } profileSlot;

    float timeHorizon{0.5f}; // in seconds, keep at least this much total time buffered in the stepper
public:
    void init(const StepperConfig& config);
    void reset();

    // Dispatch a single planned block. Non-blocking; ensure queue is available before calling.
    void dispatch(const BasePlannerBlock* block);
    void dispatchCallback(FixedFunction<void()>& callback);

    bool needMore()const;
    bool needMore(float overrideTimeHorizon)const;
    void waitQueueAvailable() const;
    bool stepperComplete() const{return stepper.isIdle();}

    ThreeAxisStepper& getStepper() { return stepper; }
};
