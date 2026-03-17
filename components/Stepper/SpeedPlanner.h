#pragma once
#include <Dispatcher.h>
#include "TypeUtil.h"
#include "esp_log.h"
// ---------------------------------------------------------------------------
// Interface: speed / velocity planner
// ---------------------------------------------------------------------------
// Owns the look-ahead block buffer.  Accepts segments, computes junction
// speeds, and runs backward + forward velocity planning passes.
// The planner facade pops blocks and dispatches them to the stepper.
class ISpeedPlanner {
public:
    virtual ~ISpeedPlanner() = default;
    virtual void init() = 0;
    virtual void reset() = 0;

    // Add a new linear motion block.  Returns false if the buffer is full.
    virtual bool addBlock(const Vec3& target, float nominalSpeed) = 0;

    // Re-run velocity planning.  committedExitSpeed constrains the first
    // block's entry speed (the speed the machine will be travelling when it
    // reaches the first un-dispatched block).
    void recalculate() { recalculateBlocks(count()); }
    virtual void recalculateBlocks(int blocksToProcess) = 0;

    virtual int count() const = 0;
    virtual int capacity() const = 0;
    virtual float distanceAtBlock(int index) const = 0;
    virtual void dispatchFront(Dispatcher& dispatcher) = 0;

    virtual void setMaxSpeed(float s) = 0;
    virtual void setMaxAcceleration(float a) = 0;
    virtual void setMaxJerk(float j) = 0;
    virtual void setJunctionDeviation(float d) = 0;

    virtual void setEndPosition(const Vec3& pos) = 0;
};

class JunctionDeviationPlanner;
class PlannerBlock : public BasePlannerBlock{
public:
    enum BlockType : uint8_t {
        Motion,    // normal motion segment
        Callback,  // zero-distance callback (transparent to planning)
        Wait,      // zero-speed dwell
    }blockType{Motion};
    JunctionDeviationPlanner *planner{nullptr};
    Vec3  target;          // absolute target position (mm)
    Vec3  direction;       // unit direction vector of movement
    float distance{0};     // scalar distance of move (mm)
    float nominalSpeed{0}; // requested feedrate along direction (mm/s)
    float maxEntrySpeed{0};// max junction speed at entry (from junction deviation)
    float entrySpeed{0};   // planned entry speed (mm/s)
    float exitSpeed{0};    // planned exit speed (mm/s)
    float waitTime{0};     // for BlockType::Wait, time to dwell at target (s)
    virtual Vec3SpeedProfile* createSpeedProfile(void* memory) const override;
};

// ---------------------------------------------------------------------------
// Concrete: Junction-deviation planner (Grbl / Marlin style)
// ---------------------------------------------------------------------------
// Uses the centripetal-acceleration model for junction speed and trapezoidal
// v² = v₀² + 2·a·d approximation for the planning passes.
class JunctionDeviationPlanner : public ISpeedPlanner {
private:
    friend class PlannerBlock;
    RingBuffer<PlannerBlock, PLANNER_BLOCK_BUFFER_SIZE> blocks;
    RingBuffer<FixedFunction<void()>, CallbackQueueSize> callbackBuffer;

    // Tracking for junction-speed computation across dispatches
    Vec3 endPosition{0};
    Vec3  previousDirection{0,0,0};
    float previousNominalSpeed{0};
    float previousExitSpeed{0};
    bool  hasPreviousBlock{false};

    int preRecalculationBlocksCount{0};
    bool needRecalculation{false};

    // Machine parameters
    float maxSpeed{100};
    float maxAcceleration{500};
    float maxJerk{5000};
    float junctionDeviation{0.05f};

    // Short-segment merging: segments whose time < minSegmentTime are
    // accumulated into a single chord until the merged time exceeds the
    // threshold (or a flush / idle / stop forces the pending block out).
    bool  pendingMerge{false};     // at least one short segment buffered
    Vec3  pendingTarget;           // most recent buffered target
    float pendingSpeed{0};         // speed of most recent buffered segment

    // Core block insertion (former addBlock body) — always pushes.
    bool pushBlock(const Vec3& target, float nominalSpeed);

    float computeMaxJunctionSpeed(const Vec3& prevDir, const Vec3& currDir,
                                  float prevNominal, float currNominal);
public:
    void init() override;
    void reset() override;

    bool addBlock(const Vec3& target, float nominalSpeed) override;
    bool addWaitBlock(float waitTime_s);
    bool addCallback(FixedFunction<void()>& cb);
    void recalculateBlocks(int blocksToProcess) override;
    float distanceAtBlock(int index) const override { return blocks[index].distance; }

    int count() const override { return blocks.size(); }
    int capacity() const override { return blocks.capacity(); }
    void dispatchFront(Dispatcher& dispatcher) override;

    void setMaxSpeed(float s) override { maxSpeed = s; }
    void setMaxAcceleration(float a) override { maxAcceleration = a; }
    void setMaxJerk(float j) override { maxJerk = j; }
    void setJunctionDeviation(float d) override { junctionDeviation = d; }

    float getMaxSpeed() const { return maxSpeed; }
    float getMaxAcceleration() const { return maxAcceleration; }
    float getMaxJerk() const { return maxJerk; }
    Vec3  getEndPosition() const { return endPosition; }

    bool hasPendingMerge() const { return pendingMerge; }
    bool flushPendingMerge();

    virtual void setEndPosition(const Vec3& pos) override;
    float calcStoppingDistance() const;
};
