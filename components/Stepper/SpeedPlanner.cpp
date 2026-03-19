#include "SpeedPlanner.h"
#include <cmath>
#include <algorithm>
#include <new>

Vec3SpeedProfile *PlannerBlock::createSpeedProfile(void *memory) const
{
    if(blockType == Motion) {
        SCurveSegmentProfile* out = new(memory) SCurveSegmentProfile();
        out->setup(direction,
                    entrySpeed,
                    nominalSpeed,
                    exitSpeed,
                    planner->maxAcceleration,
                    planner->maxJerk,
                    distance);
        return out;
    }
    else if(blockType == Wait) {
        ConstVec3SpeedProfile* out = new(memory) ConstVec3SpeedProfile();
        out->speed = Vec3{0,0,0};
        out->totalTime = waitTime;
        return out;
    }
    return nullptr;
}

// ============================================================================
// JunctionDeviationPlanner
// ============================================================================

void JunctionDeviationPlanner::init() {
    reset();
}

void JunctionDeviationPlanner::reset() {
    blocks.clear();
    callbackBuffer.clear();
    previousDirection    = Vec3{0,0,0};
    previousNominalSpeed = 0;
    previousExitSpeed    = 0;
    hasPreviousBlock     = false;

    preRecalculationBlocksCount = 0;
    needRecalculation = false;

    pendingMerge = false;
}

// ---------------------------------------------------------------------------
// Junction deviation — centripetal-acceleration model
// ---------------------------------------------------------------------------
float JunctionDeviationPlanner::computeMaxJunctionSpeed(
        const Vec3& prevDir, const Vec3& currDir,
        float prevNominal, float currNominal) {
    // cosTheta = -dot(prev, curr).  Same direction → -1, reversal → +1.
    float cosTheta = -dot(prevDir, currDir);

    if(cosTheta < -0.9999f) {
        // Nearly same direction — no speed limit from junction
        return std::min(prevNominal, currNominal);
    }
    if(cosTheta > 0.9999f) {
        // Near-180° reversal — must come to full stop
        return 0;
    }

    float sinHalfTheta = sqrtf((1.0f - cosTheta) * 0.5f);
    if(sinHalfTheta < 0.001f) return 0;

    // R = δ · sin(θ/2) / (1 - sin(θ/2))
    float R = DIV(junctionDeviation * sinHalfTheta, (1.0f - sinHalfTheta));
    float vJunction = sqrtf(maxAcceleration * R);

    return std::min(vJunction, std::min(prevNominal, currNominal));
}

// ---------------------------------------------------------------------------
// addBlock — append a new segment, with short-segment merging
// ---------------------------------------------------------------------------
// Segments whose execution time (distance / speed) is below minSegmentTime
// are accumulated: endPosition stays frozen at the last committed block so
// successive short segments naturally merge into a single chord.  When the
// accumulated chord finally exceeds the threshold (or a flush / idle / stop
// forces the pending block out via flushPendingMerge()), one merged block
// spanning the entire chord is pushed.
bool JunctionDeviationPlanner::addBlock(const Vec3& target, float nominalSpeed) {
    nominalSpeed = std::min(std::max(nominalSpeed, 0.001f), maxSpeed);

    // Compute merged distance from last committed position (endPosition).
    float mergedDistance = length(target - endPosition);
    if(mergedDistance < 1e-7f) return false;

    float time = mergedDistance / nominalSpeed;
    if(time < MIN_SEGMENT_TIME) {
        // Buffer this segment — don't advance endPosition.
        pendingTarget = target;
        pendingSpeed  = nominalSpeed;
        pendingMerge  = true;
        return true;
    }

    // Accumulated time >= threshold (or merging disabled) — push real block.
    pendingMerge = false;
    return pushBlock(target, nominalSpeed);
}

// ---------------------------------------------------------------------------
// flushPendingMerge — force-push the buffered short segment
// ---------------------------------------------------------------------------
bool JunctionDeviationPlanner::flushPendingMerge() {
    if(!pendingMerge) return false;
    if(!pushBlock(pendingTarget, pendingSpeed)){
        if(length(pendingTarget - endPosition) < 1e-7f) {
            // Target is effectively the same as endPosition — discard pending block.
            pendingMerge = false;
            return true;
        }
        return false;   // buffer full — keep pending so caller can retry
    }
    pendingMerge = false;
    return true;
}

// ---------------------------------------------------------------------------
// pushBlock — unconditional block insertion (core logic)
// ---------------------------------------------------------------------------
bool JunctionDeviationPlanner::pushBlock(const Vec3& target, float nominalSpeed) {
    if(blocks.full()) return false;
    needRecalculation = true;

    Vec3 delta = target - endPosition;
    float distance = length(delta);
    if(distance < 1e-7f) return false;
    Vec3 direction = delta / distance;
    nominalSpeed = std::min(std::max(nominalSpeed, 0.001f), maxSpeed);

    PlannerBlock block;
    block.target       = target;
    block.direction    = direction;
    block.distance     = distance;
    block.nominalSpeed = nominalSpeed;

    // Junction speed with previous segment
    if(!blocks.empty()) {
        block.maxEntrySpeed = computeMaxJunctionSpeed(
            blocks.back().direction, direction,
            blocks.back().nominalSpeed, nominalSpeed);
    } else if(hasPreviousBlock) {
        block.maxEntrySpeed = computeMaxJunctionSpeed(
            previousDirection, direction,
            previousNominalSpeed, nominalSpeed);
    } else {
        block.maxEntrySpeed = 0;  // starting from rest
    }

    block.entrySpeed = block.maxEntrySpeed;
    block.exitSpeed  = nominalSpeed;

    blocks.push(block);
    endPosition = target;
    return true;
}

bool JunctionDeviationPlanner::addWaitBlock(float waitTime_s)
{
    if(blocks.full()) return false;
    needRecalculation = true;

    PlannerBlock block;
    block.blockType = PlannerBlock::Wait;
    block.target    = endPosition;  // stays at current position
    block.distance  = 0;
    block.waitTime  = waitTime_s;

    // Inherit direction from the previous motion state for junction continuity.
    if(!blocks.empty()) {
        block.direction    = blocks.back().direction;
    } else if(hasPreviousBlock) {
        block.direction    = previousDirection;
    } else {
        block.direction    = Vec3{1,0,0};  // arbitrary, distance is 0
    }

    // Wait requires full stop: decelerate to 0 before waiting, start from 0 after.
    block.nominalSpeed  = 0;
    block.maxEntrySpeed = 0;
    block.entrySpeed    = 0;
    block.exitSpeed     = 0;

    blocks.push(block);
    return true;
}

// ---------------------------------------------------------------------------
// addCallback — insert a zero-distance callback block into the queue
// ---------------------------------------------------------------------------
bool JunctionDeviationPlanner::addCallback(FixedFunction<void()>& cb) {
    if(blocks.full()) return false;
    if(callbackBuffer.full()){
        ESP_LOGE("JunctionDeviationPlanner", "Callback buffer full, cannot add callback");
        return false;
    }
    needRecalculation = true;
    callbackBuffer.push(std::move(cb));  // store callback in separate buffer to keep it alive

    PlannerBlock block;
    block.blockType = PlannerBlock::Callback;
    block.target    = endPosition;  // stays at current position
    block.distance  = 0;

    // Inherit direction & nominalSpeed from the previous motion state so that
    // junction-speed computation sees continuity (same direction → no limit).
    if(!blocks.empty()) {
        block.direction    = blocks.back().direction;
        block.nominalSpeed = blocks.back().nominalSpeed;
    } else if(hasPreviousBlock) {
        block.direction    = previousDirection;
        block.nominalSpeed = previousNominalSpeed;
    } else {
        block.direction    = Vec3{1,0,0};  // arbitrary, distance is 0
        block.nominalSpeed = maxSpeed;
    }

    // Transparent to planning: allow any speed through
    block.maxEntrySpeed = block.nominalSpeed;
    block.entrySpeed    = block.nominalSpeed;
    block.exitSpeed     = block.nominalSpeed;

    blocks.push(block);
    return true;
}

// ---------------------------------------------------------------------------
// S-curve aware max reachable speed from startSpeed within a given distance.
// Uses binary search with computeRampDist to account for jerk-limited motion.
// ---------------------------------------------------------------------------
static float maxReachableSpeed(float startSpeed, float distance,
                               float maxAccel, float maxJerk, float upperBound)
{
    if(distance <= 0) return startSpeed;
    // Quick check: if even the upper bound fits, return it directly
    float tjHi, taHi;
    computeAccelTiming(upperBound - startSpeed, maxAccel, maxJerk, tjHi, taHi);
    float dHi = computeRampDist(startSpeed, tjHi, taHi, maxJerk);
    if(dHi <= distance) return upperBound;

    // Binary search
    float lo = startSpeed;
    float hi = upperBound;
    for(int iter = 0; iter < 15; iter++) {
        float mid = (lo + hi) * 0.5f;
        float tj, ta;
        computeAccelTiming(mid - startSpeed, maxAccel, maxJerk, tj, ta);
        float d = computeRampDist(startSpeed, tj, ta, maxJerk);
        if(d <= distance)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

// ---------------------------------------------------------------------------
// recalculate — GRBL-style partial backward + forward velocity passes
// ---------------------------------------------------------------------------
// The backward pass starts from the tail and terminates early once it reaches
// a block whose entry speed is already at its junction limit (maxEntrySpeed)
// and hasn't been marked dirty — changes further back cannot propagate past
// such a "barrier".  The forward pass only covers the affected range.
// For smooth curves (co-linear segments) this reduces per-recalculate cost
// from O(N) to nearly O(1).
// ---------------------------------------------------------------------------
void JunctionDeviationPlanner::recalculateBlocks(int blocksToProcess)
{
    blocksToProcess = std::min(blocksToProcess, (int)blocks.size());
    if(blocksToProcess == 0) return;

    if(!needRecalculation && blocksToProcess == preRecalculationBlocksCount) return;
    preRecalculationBlocksCount = blocksToProcess;
    needRecalculation = false;

    // --- Backward pass ---
    // Last block in the processing window must decelerate to 0.
    blocks[blocksToProcess - 1].exitSpeed = 0;

    // The backward pass always starts from the tail (where exitSpeed = 0)
    // and walks toward block 0.  It terminates early when a block's entry
    // speed is fully constrained by its junction limit (maxEntrySpeed) —
    // meaning blocks further toward the front are unaffected.
    for(int i = blocksToProcess - 1; i >= 0; i--) {
        float maxEntry = maxReachableSpeed(blocks[i].exitSpeed, blocks[i].distance,
                                           maxAcceleration, maxJerk,
                                           blocks[i].nominalSpeed);

        float newEntry = std::min({blocks[i].maxEntrySpeed, maxEntry, blocks[i].nominalSpeed});
        blocks[i].entrySpeed = newEntry;
        if(i > 0) {
            blocks[i-1].exitSpeed = std::min(blocks[i-1].nominalSpeed, newEntry);
        } 
    }

    // Constrain first block's entry to the committed exit speed.
    blocks[0].entrySpeed = std::min(blocks[0].entrySpeed, previousExitSpeed);
 
    // --- Forward pass ---
    for(int i = 0; i < blocksToProcess; i++) {
        float maxExit = maxReachableSpeed(blocks[i].entrySpeed, blocks[i].distance,
                                          maxAcceleration, maxJerk,
                                          blocks[i].nominalSpeed);

        blocks[i].exitSpeed = std::min(blocks[i].exitSpeed, maxExit);
        blocks[i].exitSpeed = std::min(blocks[i].exitSpeed, blocks[i].nominalSpeed);

        if(i < blocksToProcess - 1) {
            blocks[i+1].entrySpeed = std::min(blocks[i+1].entrySpeed,
                                              blocks[i].exitSpeed);
        }
    }
}

// ---------------------------------------------------------------------------
// pop_front — remove and return the oldest block
// ---------------------------------------------------------------------------
void JunctionDeviationPlanner::dispatchFront(Dispatcher& dispatcher) {
    PlannerBlock block = blocks.pop_front();
    if(block.blockType == PlannerBlock::Callback) {
        auto cb = callbackBuffer.pop_front();  // take ownership of callback
        dispatcher.dispatchCallback(cb);
    }
    else {
        block.planner = this;
        dispatcher.dispatch(&block);
    }
    // Remember direction/speed so future addBlock can compute junction speed
    // even when the buffer is empty.
    previousDirection    = block.direction;
    previousNominalSpeed = block.nominalSpeed;
    previousExitSpeed   = block.exitSpeed;
    hasPreviousBlock     = true;

    preRecalculationBlocksCount--;
}

void JunctionDeviationPlanner::setEndPosition(const Vec3 &pos)
{
    endPosition = pos;
}

float JunctionDeviationPlanner::calcStoppingDistance() const
{
    float speed = previousExitSpeed;
    if(speed <= 0) return 0;
    float tj, ta;
    float dvJerk = DIV(maxAcceleration * maxAcceleration , maxJerk);
    if(speed >= dvJerk) {
        tj = DIV(maxAcceleration , maxJerk);
        ta = DIV((speed - dvJerk) , maxAcceleration);
    } 
    else {
        tj = sqrtf(DIV(speed , maxJerk));
        ta = 0;
    }
    // S-curve ramp distance from 0→speed (== speed→0 by time reversal)
    float aEff = maxJerk * tj;
    float v1 = maxJerk * tj * tj * 0.5f;
    float d0 = maxJerk * tj * tj * tj * (1.0f / 6.0f);
    float d1 = v1 * ta + aEff * ta * ta * 0.5f;
    float v2 = v1 + aEff * ta;
    float d2 = v2 * tj + aEff * tj * tj * 0.5f - maxJerk * tj * tj * tj * (1.0f / 6.0f);
    return d0 + d1 + d2;
}