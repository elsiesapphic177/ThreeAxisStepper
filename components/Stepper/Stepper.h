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
    inline float getIntegrate(const SpeedProfile* sp, float t){ return sp->integrate(t) + velocityOffset * t; }
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

    float initSubSpeedProfile();
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

class BrentStepper: public BaseStepper{
private:
    // solve pulse timing with Brent-Dekker hybrid method (bisection + secant + inverse quadratic interpolation)
    // no derivative required, no C1 continuity requirement for speed profile
    // superlinear convergence for smooth profiles (typically 3-5 iterations vs 12-18 for pure bisection)
    // safe fallback to bisection guarantees convergence for any continuous profile
    static const char* stepperTag;

    const float tolerance{1e-6f};
    const int maxIterations{20};
    inline float findRoot(const SpeedProfile* sp, float value, float t0, float t1, bool t0Above);
public:
    BrentStepper() = default;
    virtual void resetTimer()override;
    virtual void sendPulse(const SpeedProfile* sp,int64_t initTime)override;
    virtual void sendPulse(const SubSpeedProfile& sp,float startAccumulatedStep, int64_t initTime) override;
    virtual float getPosition() const override;
    virtual void setPosition(float position) override;
};

class ImprovedBisectionStepper: public BaseStepper{
private:
    // Divide-and-conquer shared-work bisection:
    // One integrate() evaluation at the midpoint of each tree node is reused by both sub-intervals,
    // reducing total integrate() calls from O(N*20) to O((N-1) + N*k_leaf) where k_leaf ~ 3-4.
    // Typical reduction: ~70% fewer integrate() calls vs plain BisectionStepper.
    // Same robustness guarantees: no C1 continuity required, same tolerance.
    static const char* stepperTag;
    static constexpr int MAX_PULSES_PER_INTERVAL = 64;  // covers up to 64 kHz at 1 ms interval
    static constexpr int TASK_STACK_SIZE = 20;          // >= 2*ceil(log2(MAX_PULSES)) + 4
    static constexpr int LEAF_MAX_ITER = 8;
    const float tolerance{1e-6f};

    struct DivideTask { float t_a, t_b, f_a, f_b; int lo, hi; };

    inline float findLeafRoot(const SpeedProfile* sp, float target,
                              float t_a, float t_b, float f_a, float f_b);
    void solveAndEmit(const SpeedProfile* sp, int64_t baseTime, int64_t trimOffsetUs,
                      float initStep, int N, bool increaseFlag,
                      float f_start, float f_end, float totalTime);
public:
    ImprovedBisectionStepper() = default;
    virtual void resetTimer() override;
    virtual void sendPulse(const SpeedProfile* sp, int64_t initTime) override;
    virtual void sendPulse(const SubSpeedProfile& sp, float startAccumulatedStep, int64_t initTime) override;
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

    enum PulseGenType: uint8_t{
        MCPWM,
    }
    initPulseGenType{MCPWM};
    enum StepperSolver: uint8_t{
        BISECTION,
        BRENT,
        IMPROVED_BISECTION,
#if NewtonStepperAvailable
        NEWTON
#endif
    }
    initStepperSolver{BISECTION};
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
