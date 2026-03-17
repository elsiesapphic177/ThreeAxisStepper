#include "MotionProfile.h"
#include <algorithm>
#include <cmath>
#include "esp_log.h"

void SubSpeedProfile::resetTrim()
{
    trimStartTimeUs = 0;
    trimStartTime = 0;
    trimEndTimeUs = 0;
    trimEndTime = 0;
    totalTime = 0;
    totalTimeUs = static_cast<int64_t>(baseProfile ? baseProfile->totalTime * 1e6f : 0);
    for(int i=0;i<3;i++) {
        preIntegrate[i] = 0;
        endIntegrate[i] = 0;
    }
}

int64_t SubSpeedProfile::trim(int64_t deltaTimeUs)
{
    trimStartTimeUs = trimEndTimeUs;
    trimStartTime = static_cast<float>(trimEndTimeUs) * 1e-6f;
    trimEndTimeUs = trimStartTimeUs + deltaTimeUs;
    int64_t dt = 0;
    if(trimEndTimeUs > totalTimeUs){
        trimEndTimeUs = totalTimeUs;
        dt = trimStartTimeUs + deltaTimeUs - trimEndTimeUs;
    }
    trimEndTime = static_cast<float>(trimEndTimeUs) * 1e-6f;
    totalTime = trimEndTime - trimStartTime;
    for(int i=0;i<3;i++){
        preIntegrate[i] = endIntegrate[i];
        endIntegrate[i] = baseProfile->getAxisIntegrate(i, trimEndTime);
    }
    return dt;
}

float SubSpeedProfile::integrate(float t) const
{
    return baseProfile == nullptr ? 0 : baseProfile->getAxisIntegrate(axis,t + trimStartTime) - preIntegrate[axis];
}

float SubSpeedProfile::getSpeed(float t) const
{
    return baseProfile == nullptr ? 0 : baseProfile->getAxisSpeed(axis,t + trimStartTime);
}

SubSpeedProfile Vec3SpeedProfile::getSubProfile() const
{
    SubSpeedProfile profile(const_cast<Vec3SpeedProfile*>(this));
    return profile;
}

float TrapezoidalVec3SpeedProfile::calcIntegrate(float maxSpeed,float t) const
{
    if(t< 0)return 0;
    if(rampTime == 0) return maxSpeed * t;
    float acc = DIV(maxSpeed, rampTime);
    if(t < rampTime) return 0.5f * acc * t * t;
    else if( t < totalTime - rampTime) return 0.5f * acc * rampTime * rampTime + maxSpeed * (t - rampTime);
    else {
        float decT = totalTime - t;
        return (totalTime - rampTime) * maxSpeed - 0.5f * acc * decT * decT;
    }
}

float TrapezoidalVec3SpeedProfile::calcSpeed(float maxSpeed, float t) const
{
    if(t< 0)return 0;
    if(rampTime == 0) return maxSpeed;
    float acc = DIV(maxSpeed, rampTime);
    if(t < rampTime) return acc * t;
    else if( t < totalTime - rampTime) return maxSpeed;
    else {
        float decT = totalTime - t;
        return acc * decT;
    }
}

// ============================================================================
// SCurveSegmentProfile implementation
// ============================================================================

// Helper: compute jerk time and constant-accel time for a given speed change
void computeAccelTiming(float speedChange, float maxAccel, float maxJerk,
                                float& jerkTime, float& constAccelTime) {
    if(speedChange <= 0) {
        jerkTime = 0;
        constAccelTime = 0;
        return;
    }
    float tj = DIV(maxAccel, maxJerk);
    float dvJerk = DIV(maxAccel * maxAccel, maxJerk); // speed change from two jerk phases

    if(speedChange >= dvJerk) {
        jerkTime = tj;
        constAccelTime = DIV((speedChange - dvJerk), maxAccel);
    } else {
        jerkTime = std::sqrt(DIV(speedChange, maxJerk));
        constAccelTime = 0;
    }
}

// Helper: compute distance covered during a 3-phase accel/decel ramp
// (jerk_up + const_accel + jerk_down) starting from startSpeed
float computeRampDist(float startSpeed, float jerkTime, float constAccelTime, float jerkVal) {
    if(jerkTime <= 0 && constAccelTime <= 0) return 0;
    float aEff = jerkVal * jerkTime; // effective max acceleration in this ramp
    float tj = jerkTime;
    float ta = constAccelTime;

    // Phase A (jerk up): v(t)=startSpeed + J*t²/2
    float v1 = startSpeed + jerkVal * tj * tj * 0.5f;
    float d0 = startSpeed * tj + jerkVal * tj * tj * tj * (1.0f / 6.0f);

    // Phase B (const accel): v(t)=v1 + aEff*t
    float v2 = v1 + aEff * ta;
    float d1 = v1 * ta + aEff * ta * ta * 0.5f;

    // Phase C (jerk down): v(t)=v2 + aEff*t - J*t²/2
    float d2 = v2 * tj + aEff * tj * tj * 0.5f - jerkVal * tj * tj * tj * (1.0f / 6.0f);

    return d0 + d1 + d2;
}

void SCurveSegmentProfile::setup(const Vec3& dir, float entrySpd, float cruiseSpd, float exitSpd,
                                  float maxAccel, float maxJerk, float distance) {
    direction = dir;
    float entrySpeed = std::max(entrySpd, 0.0f);
    jerk = maxJerk;
    for(int i = 0; i < 8; i++) phaseStartTime[i] = 0;
    totalTime = 0;

    if(distance <= 1e-7f) return;

    exitSpd = std::max(exitSpd, 0.0f);
    cruiseSpd = std::max(cruiseSpd, std::max(entrySpeed, exitSpd));
    cruiseSpd = std::max(cruiseSpd, 1e-7f);

    // Compute accel ramp timing (phases 0-2)
    float tjA, taConst;
    computeAccelTiming(cruiseSpd - entrySpeed, maxAccel, maxJerk, tjA, taConst);
    float dAccel = computeRampDist(entrySpeed, tjA, taConst, maxJerk);

    // Compute decel ramp timing (phases 4-6) — by time-reversal symmetry,
    // decel distance from cruiseSpd→exitSpd == accel distance from exitSpd→cruiseSpd
    float tjD, tdConst;
    computeAccelTiming(cruiseSpd - exitSpd, maxAccel, maxJerk, tjD, tdConst);
    float dDecel = computeRampDist(exitSpd, tjD, tdConst, maxJerk);

    float dCruise = distance - dAccel - dDecel;

    // If not enough room for full accel+decel, reduce cruise speed via binary search
    if(dCruise < -1e-6f) {
        float lo = std::max(entrySpeed, exitSpd);
        float hi = cruiseSpd;
        for(int iter = 0; iter < 30; iter++) {
            float mid = (lo + hi) * 0.5f;
            float tja2, ta2, tjd2, td2;
            computeAccelTiming(mid - entrySpeed, maxAccel, maxJerk, tja2, ta2);
            computeAccelTiming(mid - exitSpd,    maxAccel, maxJerk, tjd2, td2);
            float da = computeRampDist(entrySpeed, tja2, ta2, maxJerk);
            float dd = computeRampDist(exitSpd,    tjd2, td2, maxJerk);
            if(da + dd > distance)
                hi = mid;
            else
                lo = mid;
        }
        cruiseSpd = lo;
        computeAccelTiming(cruiseSpd - entrySpeed, maxAccel, maxJerk, tjA, taConst);
        computeAccelTiming(cruiseSpd - exitSpd,    maxAccel, maxJerk, tjD, tdConst);
        dAccel = computeRampDist(entrySpeed, tjA, taConst, maxJerk);
        dDecel = computeRampDist(exitSpd,    tjD, tdConst, maxJerk);
        dCruise = distance - dAccel - dDecel;
        if(dCruise < 0) dCruise = 0;
    }

    // Build cumulative time boundaries directly
    phaseStartTime[0] = 0;
    phaseStartTime[1] = tjA;
    phaseStartTime[2] = phaseStartTime[1] + taConst;
    phaseStartTime[3] = phaseStartTime[2] + tjA;
    phaseStartTime[4] = phaseStartTime[3] + ((cruiseSpd > 1e-7f) ? DIV(dCruise, cruiseSpd) : 0);
    phaseStartTime[5] = phaseStartTime[4] + tjD;
    phaseStartTime[6] = phaseStartTime[5] + tdConst;
    phaseStartTime[7] = phaseStartTime[6] + tjD;
    totalTime = phaseStartTime[7];

    aAccel = jerk * tjA;
    aDecel = jerk * tjD;
    phaseStartV[0]    = entrySpeed;
    phaseStartPos[0]  = 0;

    precompute();
}

// Precompute cumulative time, speed, and position at the start of each phase.
// Called once by setup(). Avoids re-accumulating through all preceding phases
// on every calcScalarSpeed / calcScalarPos call.
void SCurveSegmentProfile::precompute() {
    for(int p = 0; p < 7; p++) {
        float dur = phaseStartTime[p + 1] - phaseStartTime[p];
        float v   = phaseStartV[p];
        float pos = phaseStartPos[p];

        float dv = 0, dp = 0;
        switch(p) {
            case 0: // accel jerk up
                dp = v * dur + jerk * dur * dur * dur * (1.0f / 6.0f);
                dv = jerk * dur * dur * 0.5f;
                break;
            case 1: // const accel
                dp = v * dur + aAccel * dur * dur * 0.5f;
                dv = aAccel * dur;
                break;
            case 2: // accel jerk down
                dp = v * dur + aAccel * dur * dur * 0.5f - jerk * dur * dur * dur * (1.0f / 6.0f);
                dv = aAccel * dur - jerk * dur * dur * 0.5f;
                break;
            case 3: // cruise
                dp = v * dur;
                dv = 0;
                break;
            case 4: // decel jerk down
                dp = v * dur - jerk * dur * dur * dur * (1.0f / 6.0f);
                dv = -(jerk * dur * dur * 0.5f);
                break;
            case 5: // const decel
                dp = v * dur - aDecel * dur * dur * 0.5f;
                dv = -(aDecel * dur);
                break;
            case 6: // decel jerk up
                dp = v * dur - aDecel * dur * dur * 0.5f + jerk * dur * dur * dur * (1.0f / 6.0f);
                dv = -(aDecel * dur - jerk * dur * dur * 0.5f);
                break;
        }
        phaseStartV[p + 1]    = v + dv;
        phaseStartPos[p + 1]  = pos + dp;
    }
}

float SCurveSegmentProfile::calcScalarSpeed(float t) const {
    if(t <= 0) return phaseStartV[0];
    if(t >= totalTime) return phaseStartV[7];

    // Find active phase (linear scan, skipping zero-duration phases)
    int p = 0;
    while(p < 6 && t >= phaseStartTime[p + 1]) p++;

    float lt = t - phaseStartTime[p];
    float v  = phaseStartV[p];

    switch(p) {
        case 0: return v + jerk * lt * lt * 0.5f;
        case 1: return v + aAccel * lt;
        case 2: return v + aAccel * lt - jerk * lt * lt * 0.5f;
        case 3: return v;
        case 4: return v - jerk * lt * lt * 0.5f;
        case 5: return v - aDecel * lt;
        case 6: return v - aDecel * lt + jerk * lt * lt * 0.5f;
        default: return v;
    }
}

float SCurveSegmentProfile::calcScalarPos(float t) const {
    if(t <= 0) return 0;
    if(t >= totalTime) return phaseStartPos[7];

    // Find active phase
    int p = 0;
    while(p < 6 && t >= phaseStartTime[p + 1]) p++;

    float lt  = t - phaseStartTime[p];
    float v   = phaseStartV[p];
    float pos = phaseStartPos[p];

    switch(p) {
        case 0: return pos + v * lt + jerk * lt * lt * lt * (1.0f / 6.0f);
        case 1: return pos + v * lt + aAccel * lt * lt * 0.5f;
        case 2: return pos + v * lt + aAccel * lt * lt * 0.5f - jerk * lt * lt * lt * (1.0f / 6.0f);
        case 3: return pos + v * lt;
        case 4: return pos + v * lt - jerk * lt * lt * lt * (1.0f / 6.0f);
        case 5: return pos + v * lt - aDecel * lt * lt * 0.5f;
        case 6: return pos + v * lt - aDecel * lt * lt * 0.5f + jerk * lt * lt * lt * (1.0f / 6.0f);
        default: return pos;
    }
}

float SCurveSegmentProfile::getAxisIntegrate(int axis, float t) const {
    float d;
    switch(axis) {
        case 0: d = direction.x; break;
        case 1: d = direction.y; break;
        case 2: d = direction.z; break;
        default: return 0;
    }
    return d * calcScalarPos(t);
}

float SCurveSegmentProfile::getAxisSpeed(int axis, float t) const {
    float d;
    switch(axis) {
        case 0: d = direction.x; break;
        case 1: d = direction.y; break;
        case 2: d = direction.z; break;
        default: return 0;
    }
    return d * calcScalarSpeed(t);
}