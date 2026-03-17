#pragma once
#include <MotionProfile.h>
#include <PulseGenerator.h>
#include <TypeUtil.h>

const int64_t DeltaTime_us = static_cast<int64_t>(DeltaTime * 1e6f);

class BaseStepper{ 
protected:
    PulseGenerator* targetPulseGen{nullptr};
    float StepUnit{0.005f};
    float StepUnitInv{200};
    float accumulatedStep{AccumulatedStepInit};

    float velocityOffset{0};// for pid control
    float accumulateVelocityOffset{0};// for pid control in subSpeedProfile
    inline float getIntegrate(const SpeedProfile* sp, float t){ return sp->integrate(t) + velocityOffset * t + accumulateVelocityOffset; }
    inline float getSpeed(const SpeedProfile* sp, float t){ return sp->getSpeed(t) + velocityOffset; }
public:
    virtual ~BaseStepper() = default;
    void init(PulseGenerator* targetPulseGen_in, float stepUnit_in);
    void setVelocityOffset(float offset);
    float getStepUnit() const { return StepUnit; }
    bool isRunning() const{ return targetPulseGen ? targetPulseGen->isRunning() : false; };

    virtual void resetTimer() = 0;
    virtual void sendPulse(const SpeedProfile* sp,int64_t initTime) = 0;
    virtual float getPosition() const = 0;
    virtual void setPosition(float position) = 0;
    PulseGenerator* getPulseGen() const { return targetPulseGen; }

    float getAccumulatedStep()const{return accumulatedStep + accumulateVelocityOffset;}
    virtual void sendPulse(const SubSpeedProfile& sp,float startAccumulatedStep, int64_t initTime) = 0;
};

#if NewtonStepperAvailable
class NewtonStepper: public BaseStepper{
private:
    // solve pulse timing using Newton's method, converges faster but less robust than BisectionStepper
    // require speed profile to be C1 (speed) continuous
    static const char* stepperTag;
    const float tolerance{1e-6f};
    const float looseTolerance{1e-3f};
    const int maxIterations{10};
    enum SolverResult: uint8_t{
        SOLVER_NON_CONVERGED,
        SOLVER_OUT_OF_END,
        SOLVER_SUCCESS,
        SOLVER_CHANGE_DIRECTION
    };
    inline float findZeroSpeed(const SpeedProfile* sp, float t0, float t1, bool t0Positive);
    SolverResult NewtonSolver(const SpeedProfile* sp, float value,float prePulseTime, float initGuess, float& nextPulseTime);
    SolverResult solveNextPulse(const SpeedProfile* sp, float initStep,float prePulseTime, float& nextPulseTime, float& nextStep, bool& increaseFlag);
public:
    NewtonStepper() = default;
    virtual void resetTimer()override;
    virtual void sendPulse(const SpeedProfile* sp,int64_t initTime)override;
    virtual void sendPulse(const SubSpeedProfile& sp,float startAccumulatedStep, int64_t initTime) override;
    virtual float getPosition() const override;
    virtual void setPosition(float position) override;
};
#endif

class BisectionStepper: public BaseStepper{
private:
    // solve pulse timing with bisection method, should be more robust but converged slower than NewtonStepper
    // although the convergence is slower, there is no division required so might be as fast as NewtonStepper
    // no C1 continuity requirement for speed profile
    // require the speed profile to be short enough so that pulse will most likely not be missed
    static const char* stepperTag;

    const float tolerance{1e-6f}; // set to 1e-5 for less accurate but as fast as NewtonStepper, set to 1e-6 for as accurate as Newton Stepper but slower
    const int maxIterations{20};
    inline float findRoot(const SpeedProfile* sp, float value, float t0, float t1, bool t0Above);
public:
    BisectionStepper() = default;
    virtual void resetTimer()override;
    virtual void sendPulse(const SpeedProfile* sp,int64_t initTime)override;
    virtual void sendPulse(const SubSpeedProfile& sp,float startAccumulatedStep, int64_t initTime) override;
    virtual float getPosition() const override;
    virtual void setPosition(float position) override;
};

struct StepperConfig{
    enum ErrorType: uint8_t{
        PID_LIMIT_ERROR,
        INTERVAL_TOO_LONG_ERROR,
        STEP_LOST_ERROR,
    };
    gpio_num_t PulsePin[3];
    gpio_num_t DirPin[3];
    bool DirInverse[3]{false, false, false};
    float StepUnit[3];
    gpio_num_t EnablePin{GPIO_NUM_NC};
    FixedFunction<void(bool)> enablePinCallback{nullptr};// callback to set enable pin, if not using gpio directly, true for on, false for off
    FixedFunction<Vec3()> rulerCallback{nullptr};
    FixedFunction<void(const Vec3&)> rulerSetPositionCallback{nullptr};
    FixedFunction<void(StepperConfig::ErrorType)> errorCallback{nullptr};
};

class ThreeAxisStepper{
private:
    // account for synchronization of three axes, pid control and non-blocking motion feeding to pulse generator.
    // trim vec3 motion into tiny speed profile (~1ms each if pid enable)
    // feed speed profile to each axis pulse generator without blocking main task
    struct Impl;
    Impl* impl{nullptr};
public:
    ThreeAxisStepper() = default;
    ~ThreeAxisStepper();
    enum PulseGenType: uint8_t{
        MCPWM,
    }
    initPulseGenType{MCPWM};
    enum StepperSolver: uint8_t{
        BISECTION,
#if NewtonStepperAvailable
        NEWTON
#endif
    }
    initStepperSolver{BISECTION};
    void init(const StepperConfig& config);

    void reset(); // immediately stop and reset all state
    void emergencyStop();// reset without changing current position. unable to resume
    void emergencyStopRelease();// call this to release the lock after emergency stop, so stepper can be used again

    void move(Vec3SpeedProfile* speedProfile, uint32_t tickDelay);
    void addCallback(FixedFunction<void()>& callback, uint32_t tickDelay);
    bool queueAvailable() const;
    bool isIdle()const;
    float getRemainingQueueTime() const;// in seconds, to see how much time left for all motion in queue to complete

    void setCurrentPosition(const Vec3& position);
    Vec3 getCurrentPosition(bool fromRulerIfPIDEnable = true);
    Vec3 getTargetPosition();

    void setEnablePID(bool enableFlag);
    void setPID(float kp_in, float ki_in, float kd_in);
    void setPIDErrorLimit(float errorLimit);
    bool isPIDEnable();
    Vec3 getPID();
    float getPIDErrorLimit();

    BaseStepper* getAxisStepper(int axis);
};
