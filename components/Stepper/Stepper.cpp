#include "Stepper.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "MemoryUtil.h"

void BaseStepper::init(PulseGenerator *targetPulseGen_in, float stepUnit_in)
{
    targetPulseGen = targetPulseGen_in;
    StepUnit = stepUnit_in;
    StepUnitInv = 1.0f / StepUnit;
}

void BaseStepper::setVelocityOffset(float offset)
{
    velocityOffset = offset;
}

float BaseStepper::initSubSpeedProfile() 
{
    accumulateVelocityOffset = 0;
    return accumulatedStep;
}

#if NewtonStepperAvailable
const char* NewtonStepper::stepperTag = "NewtonStepper";
void NewtonStepper::resetTimer()
{
    targetPulseGen->reset();
    velocityOffset = 0;
    accumulatedStep = AccumulatedStepInit;
    accumulateVelocityOffset = 0;
}

float NewtonStepper::findZeroSpeed(const SpeedProfile *sp, float t0, float t1, bool t0Positive)
{ 
    float midT = t0, v;
    for(int j = 0; j < maxIterations; j++){
        midT = (t0 + t1)/2;
        v = getSpeed(sp, midT);
        if(fabsf(v) < looseTolerance){
            return midT;
        }
        else if((t0Positive && v < 0) || (!t0Positive && v > 0)){
            t1= midT;
        }
        else{
            t0 = midT;
        }
    }
    return midT;
}

NewtonStepper::SolverResult NewtonStepper::NewtonSolver(const SpeedProfile *sp, float value,float prePulseTime, float initGuess, float &nextPulseTime)
{
    // use Newton's method to solve the time for next pulse, return time
    float t = initGuess;
    float preDf = getSpeed(sp, t);
    for(int i = 0;i< maxIterations && fabsf(preDf) < looseTolerance; i++){
        t += looseTolerance;
        preDf = getSpeed(sp, t);
    }
    if(fabsf(preDf) < looseTolerance){
        // derivative is too small
        return SOLVER_NON_CONVERGED;
    }
    initGuess = t;
    float f, df;
    for(int i=0;i<maxIterations;i++){
        f = getIntegrate(sp, t) - value;
        if(fabsf(f) < tolerance){
            nextPulseTime = t;
            if(t < prePulseTime || t > sp->totalTime)
                return SOLVER_OUT_OF_END;
            else
                return SOLVER_SUCCESS;
        }
        df = getSpeed(sp, t);
        if((preDf > 0 && df < 0) || (preDf< 0 && df > 0)){
            nextPulseTime = findZeroSpeed(sp,initGuess,t,preDf > 0) + looseTolerance; // add a small value to avoid precision error
            return SOLVER_CHANGE_DIRECTION;
        }
        if(df == 0){
            // derivative is zero ,try to perturb t a little bit and continue, this may cause some error but should be acceptable for well defined profile
            t += looseTolerance;
        }
        else{
            t = t - DIV(f, df);
        }
    }
    return SOLVER_NON_CONVERGED;
}

NewtonStepper::SolverResult NewtonStepper::solveNextPulse(const SpeedProfile *sp, float initStep,float prePulseTime, float &nextPulseTime, float &nextStep, bool &increaseFlag)
{
    increaseFlag = getSpeed(sp, prePulseTime) >= 0;
    nextStep = increaseFlag ?static_cast<int>(floorf(accumulatedStep)) + 1: static_cast<int>(ceilf(accumulatedStep)) - 1;
    SolverResult res = NewtonSolver(sp, (nextStep - initStep) * StepUnit,prePulseTime, prePulseTime, nextPulseTime);
    if(res == SOLVER_CHANGE_DIRECTION || res == SOLVER_NON_CONVERGED){
        nextStep = increaseFlag ? nextStep - 1 : nextStep + 1;
        increaseFlag = !increaseFlag;
        res = NewtonSolver(sp, (nextStep - initStep) * StepUnit, prePulseTime, nextPulseTime, nextPulseTime);
        if(res == SOLVER_CHANGE_DIRECTION)
            res = SOLVER_NON_CONVERGED;
    }
    if(res == SOLVER_SUCCESS){
        if(static_cast<int>(floorf(nextStep)) == static_cast<int>(ceilf(nextStep))){ // integer step, refine to avoid precision error
            nextStep = std::nextafterf(nextStep, increaseFlag ? nextStep + 1 : nextStep - 1);
        }
    }
    return res;
}

void NewtonStepper::sendPulse(const SpeedProfile* sp,int64_t initTime)
{
    if(targetPulseGen == nullptr || sp == nullptr || sp->totalTime <= 0) return;
    float dStep = getIntegrate(sp, sp->totalTime) * StepUnitInv;
    float finalStep = accumulatedStep + dStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){ // integer step, refine to avoid precision error
        finalStep = std::nextafterf(finalStep, getSpeed(sp, sp->totalTime) > 0 ? finalStep - 1 : finalStep + 1);
    }
    float initStep = accumulatedStep;
    float t = 0; 
    float nextStep, nextT;
    bool increaseFlag;
    while(true){
        SolverResult res = solveNextPulse(sp, initStep, t, nextT, nextStep, increaseFlag);
        if(res != SOLVER_SUCCESS){
            break;
        }
        if(fabsf(nextT - sp->totalTime) < 0.0001f) {
            //close to the end, check for whether nextStep is over final step to avoid excess pulse caused by precision error
            if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
                break;
            }
        }
        //ESP_LOGI(stepperTag, "nextStep: %f, nextT: %f", nextStep, nextT);
        targetPulseGen->addPulse(initTime + static_cast<int64_t>(nextT * 1e6f), increaseFlag);
        t = nextT;
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
}

void NewtonStepper::sendPulse(const SubSpeedProfile &sp, float startAccumulatedStep, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp.totalTime <= 0) return;
    float finalStep = startAccumulatedStep + (sp.getAxisEndIntegrate() + accumulateVelocityOffset + velocityOffset * sp.totalTime) * StepUnitInv;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){ // integer step, refine to avoid precision error
        finalStep = std::nextafterf(finalStep, getSpeed(&sp, sp.totalTime) > 0 ? finalStep - 1 : finalStep + 1);
    }
    float initStep = accumulatedStep;
    float t = 0; 
    float nextStep, nextT;
    bool increaseFlag;
    while(true){
        SolverResult res = solveNextPulse(&sp, initStep, t, nextT, nextStep, increaseFlag);
        if(res != SOLVER_SUCCESS){
            break;
        }
        if(fabsf(nextT - sp.totalTime) < 0.0001f) {
            //close to the end, check for whether nextStep is over final step to avoid excess pulse caused by precision error
            if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
                break;
            }
        }
        //ESP_LOGI(stepperTag, "nextStep: %f, nextT: %f", nextStep, nextT);
        targetPulseGen->addPulse(initTime + sp.getTrimStartTimeUs() + static_cast<int64_t>(nextT * 1e6f), increaseFlag);
        t = nextT;
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
    accumulateVelocityOffset += velocityOffset * sp.totalTime;
}

float NewtonStepper::getPosition() const
{
    return targetPulseGen ? targetPulseGen->getCount() * StepUnit : 0;
}

void NewtonStepper::setPosition(float position)
{
    if(targetPulseGen){
        targetPulseGen->setCount(static_cast<int32_t>(position * StepUnitInv));
        accumulatedStep = AccumulatedStepInit;
    }
}
#endif

const char* BisectionStepper::stepperTag = "BisectionStepper";
inline float BisectionStepper::findRoot(const SpeedProfile *sp, float value, float t0, float t1, bool t0Above)
{
    float midT = t0, v;
    for(int j = 0; j < maxIterations; j++){
        midT = (t0 + t1)/2;
        v = getIntegrate(sp, midT) - value;
        if(fabsf(v) < tolerance){
            return midT;
        }
        else if((t0Above && v < 0) || (!t0Above && v > 0)){
            t1= midT;
        }
        else{
            t0 = midT;
        }
    }
    return midT;
}

void BisectionStepper::resetTimer()
{
    targetPulseGen->reset();
    velocityOffset = 0;
    accumulatedStep = AccumulatedStepInit;
    accumulateVelocityOffset = 0;
}

void BisectionStepper::sendPulse(const SpeedProfile *sp, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp == nullptr || sp->totalTime <= 0) return;

    float finalStep = accumulatedStep + getIntegrate(sp, sp->totalTime) * StepUnitInv;
    bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){ // integer step, refine to avoid precision error
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    }
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){ // no step needed, just return
        accumulatedStep = finalStep;
        return;
    }
    float initStep = accumulatedStep;
    float t = 0;
    float nextStep;
    while(true){
        nextStep = increaseFlag ?static_cast<int>(floorf(accumulatedStep)) + 1: static_cast<int>(ceilf(accumulatedStep)) - 1;
        if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
            break;
        }
        t = findRoot(sp, (nextStep - initStep) * StepUnit, t, sp->totalTime, !increaseFlag);
        int64_t pulseTime = initTime + static_cast<int64_t>(t * 1e6f);
        //ESP_LOGI(stepperTag, "nextStep: %f, pulseTime: %lld", nextStep, pulseTime);
        targetPulseGen->addPulse(pulseTime, increaseFlag);
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
}

void BisectionStepper::sendPulse(const SubSpeedProfile &sp, float startAccumulatedStep, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp.totalTime <= 0) return;
    float finalStep = startAccumulatedStep + (sp.getAxisEndIntegrate() + accumulateVelocityOffset + velocityOffset * sp.totalTime) * StepUnitInv;
    bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){ // integer step, refine to avoid precision error
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    }
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){ // no step needed, just return
        accumulatedStep = finalStep;
        return;
    }
    float initStep = accumulatedStep;
    float t = 0;
    float nextStep;
    while(true){
        nextStep = increaseFlag ?static_cast<int>(floorf(accumulatedStep)) + 1: static_cast<int>(ceilf(accumulatedStep)) - 1;
        if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
            break;
        }
        t = findRoot(&sp, (nextStep - initStep) * StepUnit, t, sp.totalTime, !increaseFlag);
        int64_t pulseTime = initTime + sp.getTrimStartTimeUs() + static_cast<int64_t>(t * 1e6f);
        //ESP_LOGI(stepperTag, "nextStep: %f, pulseTime: %lld", nextStep, pulseTime);
        targetPulseGen->addPulse(pulseTime, increaseFlag);
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
    accumulateVelocityOffset += velocityOffset * sp.totalTime;
}

float BisectionStepper::getPosition() const
{
    return targetPulseGen ? targetPulseGen->getCount() * StepUnit : 0;
}

void BisectionStepper::setPosition(float position)
{
    if(targetPulseGen){
        targetPulseGen->setCount(static_cast<int32_t>(position * StepUnitInv));
        accumulatedStep = AccumulatedStepInit;
    }
}

// -----------------------------------------------------------------------
// BrentStepper — Brent-Dekker hybrid root-finding stepper
// Combines bisection (safe bracket) + secant (linear interpolation) +
// inverse quadratic interpolation (IQI) for superlinear convergence.
// Typically 3-5 integrate() calls per pulse vs 12-18 for pure bisection.
// -----------------------------------------------------------------------
const char* BrentStepper::stepperTag = "BrentStepper";

inline float BrentStepper::findRoot(const SpeedProfile *sp, float value, float t0, float t1, bool /*t0Above*/)
{
    // f(t) = getIntegrate(sp, t) - value
    float a = t0, b = t1;
    float fa = getIntegrate(sp, a) - value;
    float fb = getIntegrate(sp, b) - value;

    // Ensure b is the better estimate (|f(b)| <= |f(a)|)
    if(fabsf(fa) < fabsf(fb)){
        float tmp = a; a = b; b = tmp;
        tmp = fa; fa = fb; fb = tmp;
    }

    float c = a, fc = fa;
    float d = b; // last step size (unused until 2nd iteration)
    bool mflag = true;
    float s = b, fs = fb;

    for(int i = 0; i < maxIterations; i++){
        if(fabsf(fb) < tolerance || fabsf(b - a) < tolerance)
            return b;

        if(fa != fc && fb != fc){
            // Inverse quadratic interpolation
            float ab = a - b, ac = a - c, bc = b - c;
            s = DIV(a * fb * fc , fa * (-ab) * (-ac))
              + DIV(b * fa * fc , fb * ab * (-bc))
              + DIV(c * fa * fb , fc * ac * bc);
        } else {
            // Secant method
            s = b - DIV(fb * (b - a), (fb - fa));
        }

        // Five Brent conditions: fall back to bisection if any holds
        float mid = (a + b) * 0.5f;
        bool cond1 = (s < (3.0f * a + b) * 0.25f) || (s > b);
        if(a > b) cond1 = (s > (3.0f * a + b) * 0.25f) || (s < b);
        bool cond2 =  mflag && fabsf(s - b) >= fabsf(b - c) * 0.5f;
        bool cond3 = !mflag && fabsf(s - b) >= fabsf(c - d) * 0.5f;
        bool cond4 =  mflag && fabsf(b - c) < tolerance;
        bool cond5 = !mflag && fabsf(c - d) < tolerance;

        if(cond1 || cond2 || cond3 || cond4 || cond5){
            s = mid;
            mflag = true;
        } else {
            mflag = false;
        }

        fs = getIntegrate(sp, s) - value;
        d = c;
        c = b; fc = fb;

        if(fa * fs < 0.0f){
            b = s; fb = fs;
        } else {
            a = s; fa = fs;
        }

        // Keep b as the better estimate
        if(fabsf(fa) < fabsf(fb)){
            float tmp = a; a = b; b = tmp;
            tmp = fa; fa = fb; fb = tmp;
        }
    }
    return b;
}

void BrentStepper::resetTimer()
{
    targetPulseGen->reset();
    velocityOffset = 0;
    accumulatedStep = AccumulatedStepInit;
    accumulateVelocityOffset = 0;
}

void BrentStepper::sendPulse(const SpeedProfile *sp, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp == nullptr || sp->totalTime <= 0) return;

    float finalStep = accumulatedStep + getIntegrate(sp, sp->totalTime) * StepUnitInv;
    bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    }
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){
        accumulatedStep = finalStep;
        return;
    }
    float initStep = accumulatedStep;
    float t = 0;
    float nextStep;
    while(true){
        nextStep = increaseFlag ? static_cast<int>(floorf(accumulatedStep)) + 1
                                : static_cast<int>(ceilf(accumulatedStep)) - 1;
        if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
            break;
        }
        t = findRoot(sp, (nextStep - initStep) * StepUnit, t, sp->totalTime, !increaseFlag);
        int64_t pulseTime = initTime + static_cast<int64_t>(t * 1e6f);
        targetPulseGen->addPulse(pulseTime, increaseFlag);
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
}

void BrentStepper::sendPulse(const SubSpeedProfile &sp, float startAccumulatedStep, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp.totalTime <= 0) return;
    float finalStep = startAccumulatedStep + (sp.getAxisEndIntegrate() + accumulateVelocityOffset + velocityOffset * sp.totalTime) * StepUnitInv;
    bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep))){
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    }
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){
        accumulatedStep = finalStep;
        return;
    }
    float initStep = accumulatedStep;
    float t = 0;
    float nextStep;
    while(true){
        nextStep = increaseFlag ? static_cast<int>(floorf(accumulatedStep)) + 1
                                : static_cast<int>(ceilf(accumulatedStep)) - 1;
        if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)){
            break;
        }
        t = findRoot(&sp, (nextStep - initStep) * StepUnit, t, sp.totalTime, !increaseFlag);
        int64_t pulseTime = initTime + sp.getTrimStartTimeUs() + static_cast<int64_t>(t * 1e6f);
        targetPulseGen->addPulse(pulseTime, increaseFlag);
        accumulatedStep = nextStep;
    }
    accumulatedStep = finalStep;
    accumulateVelocityOffset += velocityOffset * sp.totalTime;
}

float BrentStepper::getPosition() const
{
    return targetPulseGen ? targetPulseGen->getCount() * StepUnit : 0;
}

void BrentStepper::setPosition(float position)
{
    if(targetPulseGen){
        targetPulseGen->setCount(static_cast<int32_t>(position * StepUnitInv));
        accumulatedStep = AccumulatedStepInit;
    }
}

// -----------------------------------------------------------------------
// ImprovedBisectionStepper — divide-and-conquer shared-work bisection
//
// Key idea: within one 1ms interval there are N pulses whose pulse times
// are all roots of the same monotone function integrate(t).  A plain
// bisection for each pulse independently re-evaluates the midpoint of
// the whole interval O(N) times.  By recursively splitting the time
// interval at its midpoint and classifying each pulse target to either the
// left or the right sub-interval, every midpoint evaluation is shared by
// ALL pulses that still overlap that sub-interval.
//
// Cost: (N-1) internal-node evaluations + N * k_leaf leaf evaluations,
// where k_leaf ≈ 3 (1 linear-interpolation step + ~2 bisection steps).
// Plain bisection: N * 20.  D&C at 20 pulses: 19 + 60 ≈ 79 vs 400 → ~80%
// reduction.  Same tolerance (1e-6) and no continuity requirement.
// -----------------------------------------------------------------------
const char* ImprovedBisectionStepper::stepperTag = "ImprovedBisectionStepper";

inline float ImprovedBisectionStepper::findLeafRoot(
    const SpeedProfile* sp, float target,
    float t_a, float t_b, float f_a, float f_b)
{
    if(fabsf(f_b - f_a) < 1e-12f) return (t_a + t_b) * 0.5f;
    const bool f_a_low = (f_a <= target);
    // Linear interpolation as initial probe (false-position / regula falsi).
    // For nearly-constant-speed intervals this is essentially exact on the
    // first try, reducing leaf cost to 1-2 integrate() calls.
    float t = t_a + (target - f_a) / (f_b - f_a) * (t_b - t_a);
    if(t <= t_a || t >= t_b) t = (t_a + t_b) * 0.5f;  // clamp to bracket
    for(int j = 0; j < LEAF_MAX_ITER; j++){
        const float ft = getIntegrate(sp, t);
        if(fabsf(ft - target) < tolerance) return t;
        if((ft < target) == f_a_low){ t_a = t; f_a = ft; }
        else                        { t_b = t; f_b = ft; }
        t = (t_a + t_b) * 0.5f;  // subsequent probes: pure bisection
    }
    return t;
}

void ImprovedBisectionStepper::solveAndEmit(
    const SpeedProfile* sp, int64_t baseTime, int64_t trimOffsetUs,
    float initStep, int N, bool increaseFlag,
    float f_start, float f_end, float totalTime)
{
    // Build the sorted target array (ascending for increasing, descending for decreasing).
    // targets[k] = displacement value at which pulse k must fire.
    float targets[MAX_PULSES_PER_INTERVAL];
    const float firstNextStepF = increaseFlag
        ? static_cast<float>(static_cast<int>(floorf(initStep)) + 1)
        : static_cast<float>(static_cast<int>(ceilf(initStep))  - 1);
    for(int k = 0; k < N; k++){
        const float stepK = increaseFlag ? firstNextStepF + k : firstNextStepF - k;
        targets[k] = (stepK - initStep) * StepUnit;
    }

    // Iterative D&C via explicit LIFO stack.
    // Right sub-task is pushed before left so that left is popped first,
    // ensuring pulses are emitted in chronological order.
    DivideTask taskStack[TASK_STACK_SIZE];
    int stackTop = 0;
    taskStack[stackTop++] = {0.0f, totalTime, f_start, f_end, 0, N - 1};

    while(stackTop > 0){
        const DivideTask cur = taskStack[--stackTop];
        if(cur.lo > cur.hi) continue;

        if(cur.lo == cur.hi){
            // Leaf: exactly one pulse in this bracket — solve with linear interp + bisection.
            const float t_pulse = findLeafRoot(sp, targets[cur.lo],
                                               cur.t_a, cur.t_b, cur.f_a, cur.f_b);
            targetPulseGen->addPulse(
                baseTime + trimOffsetUs + static_cast<int64_t>(t_pulse * 1e6f), increaseFlag);
            continue;
        }

        // Internal node: evaluate midpoint once, shared by both sub-trees.
        const float t_mid = (cur.t_a + cur.t_b) * 0.5f;
        const float f_mid = getIntegrate(sp, t_mid);

        // Binary-search split index k: targets[lo..k-1] belong to left, [k..hi] to right.
        // For increasing: left sub-tree handles targets <= f_mid (earlier pulse times).
        // For decreasing: left sub-tree handles targets >= f_mid.
        int lo_k = cur.lo, hi_k = cur.hi + 1;
        while(lo_k < hi_k){
            const int mid_k = (lo_k + hi_k) >> 1;
            if(increaseFlag ? targets[mid_k] <= f_mid : targets[mid_k] >= f_mid)
                lo_k = mid_k + 1;
            else
                hi_k = mid_k;
        }
        const int k = lo_k;  // left: [cur.lo, k-1], right: [k, cur.hi]

        if(k       <= cur.hi) taskStack[stackTop++] = {t_mid, cur.t_b, f_mid, cur.f_b, k,       cur.hi};
        if(cur.lo  <= k - 1)  taskStack[stackTop++] = {cur.t_a, t_mid, cur.f_a, f_mid, cur.lo, k - 1};
    }
}

void ImprovedBisectionStepper::resetTimer()
{
    targetPulseGen->reset();
    velocityOffset = 0;
    accumulatedStep = AccumulatedStepInit;
    accumulateVelocityOffset = 0;
}

void ImprovedBisectionStepper::sendPulse(const SpeedProfile* sp, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp == nullptr || sp->totalTime <= 0) return;

    const float f_end = getIntegrate(sp, sp->totalTime);
    float finalStep = accumulatedStep + f_end * StepUnitInv;
    const bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep)))
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){
        accumulatedStep = finalStep;
        return;
    }

    const float initStep = accumulatedStep;
    const int N = increaseFlag
        ? static_cast<int>(floorf(finalStep)) - static_cast<int>(floorf(initStep))
        : static_cast<int>(ceilf(initStep))   - static_cast<int>(ceilf(finalStep));

    if(N > 0 && N <= MAX_PULSES_PER_INTERVAL){
        solveAndEmit(sp, initTime, 0LL, initStep, N, increaseFlag,
                     0, f_end, sp->totalTime);
    } else {
        // Fallback: plain bisection (N=0 or N>MAX is not expected in normal use)
        float t = 0;
        while(true){
            const float nextStep = increaseFlag
                ? static_cast<float>(static_cast<int>(floorf(accumulatedStep)) + 1)
                : static_cast<float>(static_cast<int>(ceilf(accumulatedStep))  - 1);
            if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)) break;
            const float val = (nextStep - initStep) * StepUnit;
            const bool t0Above = !increaseFlag;
            float ta = t, tb = sp->totalTime, midT = ta;
            for(int j = 0; j < 20; j++){
                midT = (ta + tb) * 0.5f;
                const float v = getIntegrate(sp, midT) - val;
                if(fabsf(v) < tolerance) break;
                if((t0Above && v < 0) || (!t0Above && v > 0)) tb = midT; else ta = midT;
            }
            t = midT;
            targetPulseGen->addPulse(initTime + static_cast<int64_t>(t * 1e6f), increaseFlag);
            accumulatedStep = nextStep;
        }
    }
    accumulatedStep = finalStep;
}

void ImprovedBisectionStepper::sendPulse(const SubSpeedProfile& sp, float startAccumulatedStep, int64_t initTime)
{
    if(targetPulseGen == nullptr || sp.totalTime <= 0) return;

    const float f_end = sp.getAxisEndIntegrate() + accumulateVelocityOffset + velocityOffset * sp.totalTime;
    float finalStep = startAccumulatedStep + f_end * StepUnitInv;
    const bool increaseFlag = finalStep > accumulatedStep;
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(ceilf(finalStep)))
        finalStep = std::nextafterf(finalStep, increaseFlag ? finalStep - 1 : finalStep + 1);
    if(static_cast<int>(floorf(finalStep)) == static_cast<int>(floorf(accumulatedStep))){
        accumulatedStep = finalStep;
        return;
    }

    const float initStep = accumulatedStep;
    const int N = increaseFlag
        ? static_cast<int>(floorf(finalStep)) - static_cast<int>(floorf(initStep))
        : static_cast<int>(ceilf(initStep))   - static_cast<int>(ceilf(finalStep));

    if(N > 0 && N <= MAX_PULSES_PER_INTERVAL){
        solveAndEmit(&sp, initTime, sp.getTrimStartTimeUs(), initStep, N, increaseFlag,
                     0, f_end, sp.totalTime);
    } else {
        float t = 0;
        while(true){
            const float nextStep = increaseFlag
                ? static_cast<float>(static_cast<int>(floorf(accumulatedStep)) + 1)
                : static_cast<float>(static_cast<int>(ceilf(accumulatedStep))  - 1);
            if((increaseFlag && nextStep > finalStep) || (!increaseFlag && nextStep < finalStep)) break;
            const float val = (nextStep - initStep) * StepUnit;
            const bool t0Above = !increaseFlag;
            float ta = t, tb = sp.totalTime, midT = ta;
            for(int j = 0; j < 20; j++){
                midT = (ta + tb) * 0.5f;
                const float v = getIntegrate(&sp, midT) - val;
                if(fabsf(v) < tolerance) break;
                if((t0Above && v < 0) || (!t0Above && v > 0)) tb = midT; else ta = midT;
            }
            t = midT;
            targetPulseGen->addPulse(initTime + sp.getTrimStartTimeUs() + static_cast<int64_t>(t * 1e6f), increaseFlag);
            accumulatedStep = nextStep;
        }
    }
    accumulatedStep = finalStep;
    accumulateVelocityOffset += velocityOffset * sp.totalTime;
}

float ImprovedBisectionStepper::getPosition() const
{
    return targetPulseGen ? targetPulseGen->getCount() * StepUnit : 0;
}

void ImprovedBisectionStepper::setPosition(float position)
{
    if(targetPulseGen){
        targetPulseGen->setCount(static_cast<int32_t>(position * StepUnitInv));
        accumulatedStep = AccumulatedStepInit;
    }
}

struct ThreeAxisStepper::Impl{
    gpio_num_t enablePin{GPIO_NUM_NC};
    FixedFunction<void(bool)> enablePinCallback{nullptr};// callback to set enable pin, if not using gpio directly
    PulseGenerator *pulseGen[3]{nullptr,nullptr,nullptr};
    BaseStepper* axisGen[3]{nullptr,nullptr,nullptr};
    ~Impl(){
        for(int i=0;i<3;i++){
            if(pulseGen[i]){
                delete pulseGen[i];
                pulseGen[i] = nullptr;
            }
            if(axisGen[i]){
                delete axisGen[i];
                axisGen[i] = nullptr;
            }
        }
    }

    struct CallbackStruct{
        FixedFunction<void()> callback;
        int64_t executeTime; // in microseconds
    };
    esp_timer_handle_t stepperTimer, callbackTimer, disableMotorTimer;
    MoveQueue<CallbackStruct, CallbackQueueSize> callbackQueue;
    MoveQueue<CallbackStruct, CallbackQueueSize>::Handle currentCallback;
    SemaphoreHandle_t callbackBusySem; // binary semaphore: given = idle, taken = busy
    FixedSlotQueue<Vec3SpeedProfile,MaxProfileSize,MotionQueueSize> motionQueue;
    int lastTimerCountDown{0};

    int32_t remainingQueueTime{0}; // track the remaining total time when all motion queue complete
    SemaphoreHandle_t remainingQueueTimeMutex;
    void addRemainingQueueTime(int32_t time){
        xSemaphoreTake(remainingQueueTimeMutex, portMAX_DELAY);
        remainingQueueTime += time;
        xSemaphoreGive(remainingQueueTimeMutex);
    }
    
    FixedSlotQueue<Vec3SpeedProfile,MaxProfileSize,MotionQueueSize>::Handle currentProfile;
    SubSpeedProfile subProfile;

    int64_t startTime{0}; // start time of currentProfile start
    Vec3 startPosition{0,0,0}; // position of currentProfile start
    float accumulatedStep[3]{0,0,0}; // accumulated step for each axis at start of profile
    
    Vec3 targetPosition{0,0,0};// current position used to pid control
    Vec3 delayPosition{0,0,0}; // temporary store postPosition to update current position
    Vec3 postPosition{0,0,0}; // position 2 * DeltaTime after current position

    
    bool emergencyStopLock{false};// to avoid calling move when emergency stop is triggered
    // for pid control
    FixedFunction<Vec3()> rulerCallback{nullptr};
    FixedFunction<void(const Vec3&)> rulerSetPositionCallback{nullptr};
    FixedFunction<void(StepperConfig::ErrorType)> errorCallback{nullptr};
    Vec3 stepLostLimit{0};
    bool enablePID{false};
    float pidErrorLimit{1.0f};
    Vec3 windupValue{0,0,0}; // step unit * MaxIntegralErrorSteps
    Vec3 pidOuptuMax{0,0,0}; // step unit * MaxVelocityOffsetSteps
    Vec3 lastError{0,0,0};
    Vec3 integralError{0,0,0};
    float kp{0}, ki{0}, kd{0};
    Vec3 calcPID(const Vec3& error);
    void setEnablePID(bool enableFlag);
    void setPID(float kp_in, float ki_in, float kd_in);

    static void disableMotorTask(void* arg);
    int64_t lastInvokeTime{0};
    static void stepperTask(void* arg);
    void stepperTaskErrorHandle(StepperConfig::ErrorType errorType);
    StepperConfig::ErrorType lastErrorType;
    static void callbackTask(void* arg);
    void reset();
    void move(Vec3SpeedProfile* speedProfile,uint32_t tickDelay);
    void addCallback(FixedFunction<void()>& callback, uint32_t tickDelay);
    void setCurrentPosition(const Vec3& position);
    Vec3 getCurrentPosition(bool fromRulerIfPIDEnable = true);

    bool isIdle() const{return esp_timer_is_active(stepperTimer) == false && uxSemaphoreGetCount(callbackBusySem) > 0;}
};


static inline float clampf(float v, float maxValue) { return v < -maxValue ? -maxValue : (v > maxValue ? maxValue : v); }

Vec3 ThreeAxisStepper::Impl::calcPID(const Vec3 &error)
{
    // dt is DeltaTime, but since it is constant it can be absorbed into the kp, ki, kd 
    // when setting the PID parameters, so we don't need to calculate it here
   
    // Tentatively accumulate integral
    Vec3 candidateIntegral = integralError + error;
    // Clamp integral to prevent windup
    candidateIntegral.x = clampf(candidateIntegral.x, windupValue.x);
    candidateIntegral.y = clampf(candidateIntegral.y, windupValue.y);
    candidateIntegral.z = clampf(candidateIntegral.z, windupValue.z);

    // --- Compute total output ---
    Vec3 output = error * kp + candidateIntegral * ki + (error - lastError) * kd;

    // --- Back-calculation anti-windup ---
    // If output would be clamped by setVelocityOffset, freeze the integral
    // to prevent further accumulation in the saturated direction
    bool xSat = fabsf(output.x) > pidOuptuMax.x;
    bool ySat = fabsf(output.y) > pidOuptuMax.y;
    bool zSat = fabsf(output.z) > pidOuptuMax.z;

    // Only update integral for axes that are NOT saturated,
    // or where the error would reduce the integral (allow unwinding)
    if (!xSat || (error.x * candidateIntegral.x < 0))
        integralError.x = candidateIntegral.x;
    if (!ySat || (error.y * candidateIntegral.y < 0))
        integralError.y = candidateIntegral.y;
    if (!zSat || (error.z * candidateIntegral.z < 0))
        integralError.z = candidateIntegral.z;

    // Clamp output to max velocity offset
    output.x = clampf(output.x, pidOuptuMax.x);
    output.y = clampf(output.y, pidOuptuMax.y);
    output.z = clampf(output.z, pidOuptuMax.z);

    lastError = error;
    return output;
}

void ThreeAxisStepper::Impl::setEnablePID(bool enableFlag)
{
    if(enableFlag == enablePID) return;
    enablePID = enableFlag;
    if(!enablePID){
        // reset pid when disabling to avoid large overshoot when re-enabling
        lastError = Vec3{0,0,0};
        integralError = Vec3{0,0,0};
        for(int i=0;i<3;i++){
            axisGen[i]->setVelocityOffset(0);
        }
    }
}

void ThreeAxisStepper::Impl::setPID(float kp_in, float ki_in, float kd_in)
{
    // compensate for DeltaTime in PID parameters, so it will be time independent
    kp = kp_in;
    ki = ki_in * DeltaTime;
    kd = kd_in / DeltaTime;
}

void ThreeAxisStepper::Impl::disableMotorTask(void *arg)
{
#if !GPIO_OUTPUT_LOCK
    Impl* impl = (Impl*)arg;
    if(impl->enablePin != GPIO_NUM_NC){
        gpio_set_level(impl->enablePin, 1);// disable driver
    }
    else if(impl->enablePinCallback){
        impl->enablePinCallback(false);
    }
#endif
}

StepperDutySignalConfig();
void ThreeAxisStepper::Impl::stepperTask(void *arg)
{
    StepperDutySignalStart();
    StepperDutySignalOn();
    ThreeAxisStepper::Impl* stepper = (ThreeAxisStepper::Impl*)arg;
    if(stepper->lastInvokeTime == 0){// first time invoke
        stepper->startTime = esp_timer_get_time() + DeltaTime_us;
        for(int i=0;i<3;i++){
            stepper->axisGen[i]->resetTimer();
        }
        stepper->lastInvokeTime = esp_timer_get_time();
        stepper->subProfile.resetTrim();
    }
    if(stepper->lastTimerCountDown == 0 && esp_timer_get_time() > stepper->lastInvokeTime + static_cast<int64_t>(DeltaTime_us * 1.5f)){
        // This means the timer callback was delayed for some reason (e.g. long GC pause, or other high priority task), we should consider this as an emergency stop to avoid sudden jump of the stepper when the timer resumes
        stepper->stepperTaskErrorHandle(StepperConfig::ErrorType::INTERVAL_TOO_LONG_ERROR);
        ESP_LOGE("ThreeAxisStepper", "Timer delayed for too long, triggering emergency stop! current time: %lld, last invoke time: %lld", esp_timer_get_time(), stepper->lastInvokeTime);
        ESP_LOGE("ThreeAxisStepper","Try to reduce the load of main task, increase the timer interval or increase interrupt priority.");
        StepperDutySignalOff();
        return;
    }
    stepper->lastInvokeTime = esp_timer_get_time();
    stepper->targetPosition = stepper->delayPosition;
    stepper->delayPosition = stepper->postPosition;
    if(stepper->lastTimerCountDown != 0){
        // Check if new work arrived during countdown (race between move() push and timer stop)
        stepper->currentProfile = std::move(stepper->motionQueue.acquire());
        if(stepper->currentProfile.isValid()){
            // New profile available — cancel countdown and resume processing
            stepper->lastTimerCountDown = 0;
            stepper->startTime = esp_timer_get_time();
            stepper->startPosition = stepper->postPosition;
            stepper->subProfile = stepper->currentProfile->getSubProfile();
            // Fall through to normal processing
        }
        else{
            if(stepper->lastTimerCountDown == 1)
                stepper->lastTimerCountDown ++;
            else{
                esp_timer_start_once(stepper->disableMotorTimer, DISABLE_MOTOR_WAIT_TIME); // delay disable pin to ensure all pulses are sent before disabling driver
                esp_timer_stop(stepper->stepperTimer);
            }
            StepperDutySignalOff();
            return;
        }
    }
    // pid
    if(stepper->enablePID && stepper->rulerCallback != nullptr){
        Vec3 error = stepper->targetPosition - stepper->rulerCallback();
        if(fabsf(error.x) > stepper->pidErrorLimit || fabsf(error.y) > stepper->pidErrorLimit || fabsf(error.z) > stepper->pidErrorLimit){
            stepper->stepperTaskErrorHandle(StepperConfig::ErrorType::PID_LIMIT_ERROR);
            ESP_LOGE("ThreeAxisStepper", "PID error limit reached, target pos: (%f, %f, %f)", stepper->targetPosition.x, stepper->targetPosition.y, stepper->targetPosition.z);
            Vec3 rulerPos = stepper->rulerCallback();
            ESP_LOGE("ThreeAxisStepper", "ruler pos: (%f, %f, %f)", rulerPos.x, rulerPos.y, rulerPos.z);
            StepperDutySignalOff();
            return;
        }
        Vec3 pidOutput = stepper->calcPID(stepper->targetPosition - stepper->rulerCallback());
        stepper->axisGen[0]->setVelocityOffset(pidOutput.x);
        stepper->axisGen[1]->setVelocityOffset(pidOutput.y);
        stepper->axisGen[2]->setVelocityOffset(pidOutput.z);
    }
    else{
        Vec3 error = stepper->targetPosition - stepper->getCurrentPosition(false);
        if(fabsf(error.x)> stepper->stepLostLimit.x || fabsf(error.y) > stepper->stepLostLimit.y || fabsf(error.z) > stepper->stepLostLimit.z){
            stepper->stepperTaskErrorHandle(StepperConfig::ErrorType::STEP_LOST_ERROR);
            ESP_LOGE("ThreeAxisStepper", "Step lost error limit reached, target pos: (%f, %f, %f)", stepper->targetPosition.x, stepper->targetPosition.y, stepper->targetPosition.z);
            Vec3 currentPos = stepper->getCurrentPosition(false);
            ESP_LOGE("ThreeAxisStepper", "current pos: (%f, %f, %f)", currentPos.x, currentPos.y, currentPos.z);
            StepperDutySignalOff();
            return;
        }
    }

    int64_t dt = DeltaTime_us;
    stepper->addRemainingQueueTime(-DeltaTime_us);
    while(true){
        if(stepper->currentProfile.isValid() && !stepper->subProfile.isTrimComplete()){
            dt = stepper->subProfile.trim(dt);
            for(int i=0;i<3;i++){
                stepper->subProfile.setAxis(i);
                stepper->axisGen[i]->sendPulse(stepper->subProfile, stepper->accumulatedStep[i] ,stepper->startTime);
            }
            stepper->postPosition = stepper->startPosition + stepper->subProfile.getEndIntegrate();
            if(!stepper->subProfile.isTrimComplete()){
                break;
            }
        }
        // must release current profile BEFORE acquiring next
        stepper->currentProfile.reset();
        stepper->currentProfile = std::move(stepper->motionQueue.acquire());
        if(!stepper->currentProfile.isValid()){
            stepper->lastTimerCountDown = 1;
            break;
        }
        stepper->startTime += stepper->subProfile.getTrimEndTimeUs();
        stepper->startPosition = stepper->postPosition;
        stepper->subProfile = stepper->currentProfile->getSubProfile();
        stepper->subProfile.resetTrim();
        for(int i=0;i<3;i++){
            stepper->accumulatedStep[i] = stepper->axisGen[i]->initSubSpeedProfile();
        }
    }
    StepperDutySignalOff();
}

void ThreeAxisStepper::Impl::stepperTaskErrorHandle(StepperConfig::ErrorType errorType)
{ 
    emergencyStopLock = true;
    esp_timer_stop(stepperTimer);
    esp_timer_stop(callbackTimer);
    for(int i=0;i<3;i++){
        if(axisGen[i])
            axisGen[i]->resetTimer();
    }
    lastErrorType = errorType;
    xTaskCreate([](void* arg){
        ThreeAxisStepper::Impl* stepper = (ThreeAxisStepper::Impl*)arg;
        stepper->reset();
        if(stepper->errorCallback)
            stepper->errorCallback(stepper->lastErrorType);
        vTaskDelete(nullptr);
    }, "ThreeAxisErrorHandler", 6000, this, 3, nullptr);
}

void ThreeAxisStepper::Impl::callbackTask(void *arg)
{
    StepperDutySignalStart();
    StepperDutySignalOn();
    ThreeAxisStepper::Impl* stepper = (ThreeAxisStepper::Impl*)arg;
    while(true){
        if(stepper->currentCallback.isValid()){
            int64_t dt = stepper->currentCallback->executeTime - esp_timer_get_time();
            if(dt <= DeltaTime_us){
                stepper->currentCallback->callback();
                stepper->currentCallback.reset();
            }
            else{
                esp_timer_start_once(stepper->callbackTimer, dt);
                break;
            }
        }
        stepper->currentCallback = std::move(stepper->callbackQueue.acquire());
        if(!stepper->currentCallback.isValid()){
            xSemaphoreGive(stepper->callbackBusySem); // mark idle
            break;
        }
    }
    StepperDutySignalOff();
}

void ThreeAxisStepper::Impl::reset()
{
    esp_timer_stop(stepperTimer);
    esp_timer_stop(callbackTimer);

    for(int i=0;i<3;i++){
        if(axisGen[i])
            axisGen[i]->resetTimer();
    }
    while(true){
        if(axisGen[0]->isRunning() || axisGen[1]->isRunning() || axisGen[2]->isRunning()){
            vTaskDelay(1);
        }
        else{
            break;
        }
    }
#if !GPIO_OUTPUT_LOCK
    esp_timer_start_once(disableMotorTimer, DISABLE_MOTOR_WAIT_TIME); // delay disable pin to ensure all pulses are sent before disabling driver
#endif
    currentProfile.reset();
    motionQueue.clear(); 
    currentCallback.reset();
    callbackQueue.clear();
    xSemaphoreGive(callbackBusySem);
    lastTimerCountDown = 0;
    remainingQueueTime = 0;

    lastError = Vec3{0,0,0};
    integralError = Vec3{0,0,0};
}

void ThreeAxisStepper::Impl::move(Vec3SpeedProfile *speedProfile, uint32_t tickDelay)
{
    if(motionQueue.push(speedProfile, tickDelay)){
        addRemainingQueueTime(static_cast<int32_t>(speedProfile->totalTime * 1e6f));
    }
    if(esp_timer_is_active(stepperTimer) == false){
#if !GPIO_OUTPUT_LOCK
        esp_timer_stop(disableMotorTimer);
        if(enablePin != GPIO_NUM_NC){
            gpio_set_level(enablePin, 0);// enable driver
        }
        else if(enablePinCallback){
            enablePinCallback(true);
        }
#endif
        xSemaphoreTake(remainingQueueTimeMutex, portMAX_DELAY);
        remainingQueueTime = static_cast<int32_t>(speedProfile->totalTime* 1e6f) + DeltaTime_us;
        xSemaphoreGive(remainingQueueTimeMutex);
        lastTimerCountDown = 0;
        currentProfile.reset();
        targetPosition = getCurrentPosition();
        delayPosition = targetPosition;
        postPosition = targetPosition;
        lastInvokeTime = 0;
        esp_timer_start_periodic(stepperTimer,static_cast<uint64_t>(DeltaTime_us));
    }
}

void ThreeAxisStepper::Impl::addCallback(FixedFunction<void()>& callback, uint32_t tickDelay)
{
    if(remainingQueueTime <= DeltaTime_us){
        // if no motion or motion about to complete, execute immediately
        if(callback)
            callback();
        return;
    }
    else{
        callbackQueue.emplace(tickDelay, std::move(callback),esp_timer_get_time() + remainingQueueTime);
        if(uxSemaphoreGetCount(callbackBusySem) > 0){
            xSemaphoreTake(callbackBusySem, 0); // mark busy
            esp_timer_start_once(callbackTimer, remainingQueueTime);
        }
    }
}

void ThreeAxisStepper::Impl::setCurrentPosition(const Vec3 &position)
{
    if(esp_timer_is_active(stepperTimer) && lastTimerCountDown == 0){// on running
        ESP_LOGE("ThreeAxisStepper", "Cannot set current position while motion is running");
        return;
    }
    targetPosition = position;
    delayPosition = position;
    postPosition = position;
    startPosition = position;
    if(rulerSetPositionCallback)
        rulerSetPositionCallback(position);
    axisGen[0]->setPosition(position.x);
    axisGen[1]->setPosition(position.y);
    axisGen[2]->setPosition(position.z);
}

Vec3 ThreeAxisStepper::Impl::getCurrentPosition(bool fromRulerIfPIDEnable)
{
    if(enablePID && rulerCallback && fromRulerIfPIDEnable)
        return rulerCallback();
    else{
        Vec3 out;
        out.x = axisGen[0]->getPosition();
        out.y = axisGen[1]->getPosition();
        out.z = axisGen[2]->getPosition();
        return out;
    }
}

ThreeAxisStepper::~ThreeAxisStepper()
{
    if(impl)
        delete impl;
    impl = nullptr;
}

void ThreeAxisStepper::init(const StepperConfig &config)
{
    if(!impl)
        impl = new Impl();
    
    for(int i=0;i<3;i++){
        if(config.initPulseGenType == StepperConfig::MCPWM){
            auto pulseGen = new MCPWMPulseGenerator();
            pulseGen->init(config.PulsePin[i], config.DirPin[i], config.DirInverse[i]);
            impl->pulseGen[i] = pulseGen;
        }
        else{
            return;
        }

        if(config.initStepperSolver == StepperConfig::BISECTION){
            impl->axisGen[i] = new BisectionStepper();
        }
        else if(config.initStepperSolver == StepperConfig::BRENT){
            impl->axisGen[i] = new BrentStepper();
        }
        else if(config.initStepperSolver == StepperConfig::IMPROVED_BISECTION){
            impl->axisGen[i] = new ImprovedBisectionStepper();
        }
#if NewtonStepperAvailable
        else if(config.initStepperSolver == StepperConfig::NEWTON){
            impl->axisGen[i] = new NewtonStepper();
        }
#endif
        else{
            return;
        }
        impl->axisGen[i]->init(impl->pulseGen[i], config.StepUnit[i]);
    }
    impl->rulerCallback = config.rulerCallback;
    impl->rulerSetPositionCallback = config.rulerSetPositionCallback;
    impl->errorCallback = config.errorCallback;
    impl->windupValue = Vec3{config.StepUnit[0], config.StepUnit[1], config.StepUnit[2]} * MaxIntegralErrorSteps;
    impl->pidOuptuMax = Vec3{config.StepUnit[0], config.StepUnit[1], config.StepUnit[2]} * MaxVelocityOffsetSteps;
    impl->stepLostLimit = Vec3{config.StepUnit[0] * StepLostLimit, config.StepUnit[1] * StepLostLimit, config.StepUnit[2] * StepLostLimit};
    if(config.EnablePin != GPIO_NUM_NC){
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        io_conf.pin_bit_mask = (1ULL << config.EnablePin);
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        gpio_set_level(config.EnablePin, 1);// disable driver by default
        impl->enablePin = config.EnablePin;
    }
    else{
        impl->enablePinCallback = config.enablePinCallback;
        if(impl->enablePinCallback){
            impl->enablePinCallback(false);// disable driver by default
        }
        else{
            ESP_LOGW("ThreeAxisStepper", "No enable pin or callback provided, make sure to handle driver enable/disable in your code to avoid unexpected behavior");
        }
    }

    esp_timer_create_args_t timer_config = {
        .callback = Impl::stepperTask,
        .arg = impl,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stepperTaskTimer",
        .skip_unhandled_events = true,
    }; 
    ESP_ERROR_CHECK(esp_timer_create(&timer_config,&impl->stepperTimer));
    
    timer_config.callback = Impl::callbackTask;
    timer_config.name = "callbackTaskTimer";
    ESP_ERROR_CHECK(esp_timer_create(&timer_config,&impl->callbackTimer));

    timer_config.callback = Impl::disableMotorTask;
    timer_config.name = "disableMotorTaskTimer";
    ESP_ERROR_CHECK(esp_timer_create(&timer_config,&impl->disableMotorTimer));

    impl->remainingQueueTimeMutex = xSemaphoreCreateMutex();
    impl->callbackBusySem = xSemaphoreCreateBinary();
    xSemaphoreGive(impl->callbackBusySem); // start in idle (given) state
}

void ThreeAxisStepper::reset()
{
    if(impl)
        impl->reset();
}

void ThreeAxisStepper::emergencyStop()
{
    if(!impl)
        return;
    impl->reset();
    impl->emergencyStopLock = true;
}

void ThreeAxisStepper::emergencyStopRelease()
{
    if(impl)
        impl->emergencyStopLock = false;
}

void ThreeAxisStepper::move(Vec3SpeedProfile *speedProfile, uint32_t tickDelay)
{
    if(impl && !impl->emergencyStopLock)
        impl->move(speedProfile, tickDelay);
}

void ThreeAxisStepper::addCallback(FixedFunction<void()>& callback, uint32_t tickDelay)
{
    if(impl && !impl->emergencyStopLock)
        impl->addCallback(callback, tickDelay);
}

bool ThreeAxisStepper::queueAvailable() const
{
    return impl ? (impl->motionQueue.count() < MotionQueueSize && impl->callbackQueue.count() < CallbackQueueSize) : false;
}

bool ThreeAxisStepper::isIdle() const
{
    return impl ? impl->isIdle() : true;
}

float ThreeAxisStepper::getRemainingQueueTime()const
{
    return impl ? (float)(impl->remainingQueueTime) * 1e-6f : 0.0f;
}

void ThreeAxisStepper::setCurrentPosition(const Vec3 &position)
{
    if(impl)
        impl->setCurrentPosition(position);
}

Vec3 ThreeAxisStepper::getCurrentPosition(bool fromRulerIfPID)
{
    if(impl)
        return impl->getCurrentPosition(fromRulerIfPID);
    return Vec3{0,0,0};
}

Vec3 ThreeAxisStepper::getTargetPosition()
{
    if(impl)
        return impl->targetPosition;
    return Vec3{0,0,0};
}

void ThreeAxisStepper::setEnablePID(bool enableFlag)
{
    if(impl)
        impl->setEnablePID(enableFlag);
}

void ThreeAxisStepper::setPID(float kp_in, float ki_in, float kd_in)
{
    if(impl)
        impl->setPID(kp_in, ki_in, kd_in);
}

void ThreeAxisStepper::setPIDErrorLimit(float errorLimit)
{
    if(impl){
        impl->pidErrorLimit = errorLimit;
    }
}

bool ThreeAxisStepper::isPIDEnable()
{
    return impl ? impl->enablePID : false;
}

Vec3 ThreeAxisStepper::getPID()
{
    if(impl)
        return {impl->kp, impl->ki / DeltaTime, impl->kd * DeltaTime};
    return {0,0,0};
}

float ThreeAxisStepper::getPIDErrorLimit()
{
    return impl ? impl->pidErrorLimit : 0.0f;
}

BaseStepper *ThreeAxisStepper::getAxisStepper(int axis)
{
    if(impl && axis >= 0 && axis < 3){
        return impl->axisGen[axis];
    }
    return nullptr;
}
