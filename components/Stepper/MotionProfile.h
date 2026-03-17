#pragma once
#include "mathStructure.h"
#include <new>

#define MaxProfileSize 144 // in bytes, should be large enough to hold any profile used in the project

class SpeedProfile{
public:
    float totalTime{0}; // in seconds
    virtual ~SpeedProfile() = default;
    virtual float integrate(float t)const = 0;
    virtual float getSpeed(float t)const = 0;

    virtual void copyTo(void* dest) const = 0;
    virtual size_t getSize() const = 0;
};

template<typename DeriveClass>
class SpeedProfileDerive: public SpeedProfile{
    virtual void copyTo(void* dest) const override{ 
        static_assert(sizeof(DeriveClass) <= MaxProfileSize, "Profile size exceeds MaxProfileSize");
        ::new (dest) DeriveClass(static_cast<const DeriveClass&>(*this)); 
    }
    virtual size_t getSize() const override{ return sizeof(DeriveClass); }
};

class Vec3SpeedProfile;
class SubSpeedProfile:public SpeedProfileDerive<SubSpeedProfile>{
private:
    Vec3SpeedProfile* baseProfile{nullptr};
    int axis{0};// 0 for x, 1 for y, 2 for z

    int64_t totalTimeUs{0};

    int64_t trimStartTimeUs{0}; // source of true 
    float trimStartTime{0}; // in seconds, for calculation convenience, should be consistent with trimStartTimeUs
    int64_t trimEndTimeUs{0};
    float trimEndTime{0};

    float preIntegrate[3]{0,0,0};
    float endIntegrate[3]{0,0,0};
public:
    SubSpeedProfile() = default;
    SubSpeedProfile(Vec3SpeedProfile* baseProfile_in): baseProfile(baseProfile_in){}
    void resetTrim();
    int64_t trim(int64_t deltaTimeUs);// return remaining delta time
    bool isTrimComplete() const { return trimEndTimeUs >= totalTimeUs; }
    void setAxis(int axis_in){ axis = axis_in; }
    float getAxisEndIntegrate() const { return endIntegrate[axis]; }
    Vec3 getEndIntegrate() const { return Vec3{endIntegrate[0], endIntegrate[1], endIntegrate[2]}; }
    int64_t getTrimStartTimeUs() const { return trimStartTimeUs; }
    int64_t getTrimEndTimeUs() const {return trimEndTimeUs;}
    virtual float integrate(float t) const override;
    virtual float getSpeed(float t) const override;
};

class Vec3SpeedProfile{
private:
    virtual float getAxisIntegrate(int axis, float t) const = 0;
    virtual float getAxisSpeed(int axis, float t) const = 0;
    friend class SubSpeedProfile;
public:
    float totalTime{0}; // in seconds
    virtual ~Vec3SpeedProfile() = default;
    Vec3 integrate(float t)const { return Vec3{getAxisIntegrate(0, t), getAxisIntegrate(1, t), getAxisIntegrate(2, t)};}
    Vec3 getSpeed(float t)const { return Vec3{getAxisSpeed(0, t), getAxisSpeed(1, t), getAxisSpeed(2, t)};}
    SubSpeedProfile getSubProfile()const;

    virtual void copyTo(void* dest) const = 0;
    virtual size_t getSize() const = 0;
};

template<typename DeriveClass>
class Vec3SpeedProfileDerive: public Vec3SpeedProfile{
public:
    virtual void copyTo(void* dest) const override{ 
        static_assert(sizeof(DeriveClass) <= MaxProfileSize, "Profile size exceeds MaxProfileSize");
        ::new (dest) DeriveClass(static_cast<const DeriveClass&>(*this)); 
    }
    virtual size_t getSize() const override{ return sizeof(DeriveClass); }
};

class ConstSpeedProfile: public SpeedProfileDerive<ConstSpeedProfile>{
public:
    float speed{0};
    float integrate(float t) const override{ return speed * t;}
    float getSpeed(float t) const override{ return speed;}
};

class RampSpeedProfile: public SpeedProfileDerive<RampSpeedProfile>{
public:
    float initSpeed{0};
    float acc{0};
    float integrate(float t) const override{return initSpeed * t + 0.5f * acc * t * t;}
    float getSpeed(float t) const override{return initSpeed + acc * t;}
};

class ConstVec3SpeedProfile: public Vec3SpeedProfileDerive<ConstVec3SpeedProfile>{
public:
    Vec3 speed{0,0,0};
private:
    virtual float getAxisIntegrate(int axis, float t) const override{ 
        switch(axis){
            case 0: return speed.x * t;
            case 1: return speed.y * t;
            case 2: return speed.z * t;
            default: return 0;
        }
    }
    virtual float getAxisSpeed(int axis, float t) const override{ 
        switch(axis){
            case 0: return speed.x;
            case 1: return speed.y;
            case 2: return speed.z;
            default: return 0;
        }
    }
};

class TrapezoidalVec3SpeedProfile: public Vec3SpeedProfileDerive<TrapezoidalVec3SpeedProfile>{
public:
    Vec3 maxSpeed{0,0,0};
    float rampTime{0};
private:
    float calcIntegrate(float maxSpeed,float t)const;
    float calcSpeed(float maxSpeed,float t)const;
    virtual float getAxisIntegrate(int axis, float t) const override{ 
        switch(axis){
            case 0: return calcIntegrate(maxSpeed.x, t);
            case 1: return calcIntegrate(maxSpeed.y, t);
            case 2: return calcIntegrate(maxSpeed.z, t);
            default: return 0;
        }
    }
    virtual float getAxisSpeed(int axis, float t) const override{ 
        switch(axis){
            case 0: return calcSpeed(maxSpeed.x, t);
            case 1: return calcSpeed(maxSpeed.y, t);
            case 2: return calcSpeed(maxSpeed.z, t);
            default: return 0;
        }
    }
};

// 7-phase S-curve motion profile for a linear segment.
// Phases: [jerk_up | const_accel | jerk_down | cruise | jerk_down | const_decel | jerk_up]
// Supports asymmetric entry/exit speeds with jerk-limited acceleration.
class SCurveSegmentProfile: public Vec3SpeedProfileDerive<SCurveSegmentProfile>{
private:
    Vec3 direction{0,0,0};       // unit direction of movement
    float jerk{0};                // jerk magnitude (mm/s³)

    // Precomputed boundary tables (filled by setup):
    float phaseStartTime[8]{};   // cumulative time at start of each phase (index 7 = totalTime)
    float phaseStartV[8]{};      // scalar speed at start of each phase
    float phaseStartPos[8]{};    // scalar position at start of each phase
    float aAccel{0};              // effective max accel in accel ramp  = jerk * phaseDur[0]
    float aDecel{0};              // effective max accel in decel ramp  = jerk * phaseDur[4]
public:
    SCurveSegmentProfile() = default;

    // Setup the profile for a linear segment.
    // dir: unit direction vector
    // entrySpd/cruiseSpd/exitSpd: scalar speeds along direction (mm/s)
    // maxAccel: maximum acceleration magnitude (mm/s²)
    // maxJerk: maximum jerk magnitude (mm/s³)
    // distance: total scalar distance (mm)
    void setup(const Vec3& dir, float entrySpd, float cruiseSpd, float exitSpd,
               float maxAccel, float maxJerk, float distance);

private:
    void precompute();  // fill phaseStartTime/V/Pos tables
    float calcScalarSpeed(float t) const;
    float calcScalarPos(float t) const;
    virtual float getAxisIntegrate(int axis, float t) const override;
    virtual float getAxisSpeed(int axis, float t) const override;
};

// S-curve ramp helpers (shared by MotionProfile and SpeedPlanner)
void  computeAccelTiming(float speedChange, float maxAccel, float maxJerk,
                         float& jerkTime, float& constAccelTime);
float computeRampDist(float startSpeed, float jerkTime, float constAccelTime, float jerkVal);

