#pragma once
#include <SpeedPlanner.h>
#include "memoryUtil.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

enum JogAxis : int8_t {
    JOG_X_POS = 0, JOG_X_NEG,
    JOG_Y_POS,     JOG_Y_NEG,
    JOG_Z_POS,     JOG_Z_NEG,
};

class Planner {
private:
    // State machine: IDLE ↔ NORMAL ↔ FLUSHING, STOPPING → STOPPED (graceful stop).
    // RESET is transient — plannerTask acknowledges it and returns to IDLE.
    // NORMAL→IDLE atomically verifies moveQueue is empty (prevents lost wakeups).
    enum PlannerState: uint8_t {
        IDLE,
        NORMAL,
        FLUSHING,
        STOPPING,
        STOPPED,
        JOGGING,
        JOG_DECEL,
    } state{IDLE};
    PlannerState preStopState{NORMAL};// NORMAL or FLUSHING
    SemaphoreHandle_t stateMutex{nullptr}; // protects info
    void setState(PlannerState newState); // only call in task
    PlannerState getState(); // only call outside task
    void checkNotify(TickType_t timeout);

    JunctionDeviationPlanner speedPlanner;
    Dispatcher dispatcher;

    struct Command {
        enum Type:uint8_t{
            Move,
            Flush,
            Callback,
            Wait,
        } type{Move};
        Vec3 position; // ignored if not Move
        float speed;// use as time_s for Wait
    };
    MoveQueue<FixedFunction<void()>, CallbackQueueSize> callbackBuffer;
    QueueHandle_t moveQueue{nullptr};
    TaskHandle_t plannerTaskHandle{nullptr};
    static void plannerTask(void* arg);

    void addBlock(Command& cmd);
    void addCallbackBlock(FixedFunction<void()>& cb);
    void flushBlock();

    // --- Jog state ---
    JogAxis  jogAxis{JOG_X_POS};
    float    jogSpeed{10.0f};        // mm/s (always positive)
    int64_t  jogLastCallTime{0};     // esp_timer_get_time() of last jog() call
    void jogAccelSegment();   // dispatch 0→jogSpeed ramp
    void jogCruiseSegment();  // dispatch constant-speed segment
    void jogDecelSegment();   // dispatch jogSpeed→0 ramp, wait for idle
    Vec3 jogDirection() const; // unit direction from jogAxis
public:
    void init(const StepperConfig& config);
    void reset();// note that reset doesn't reset position

    // G-code style interface: move to absolute position at given feedrate (mm/s).
    // Blocks if the internal buffer is full (back-pressure from stepper).
    bool moveTo(const Vec3& position, float speed, TickType_t timeout = portMAX_DELAY); 
    bool addWait(float time_s, TickType_t timeout = portMAX_DELAY);
    bool addCallback(FixedFunction<void()> cb, TickType_t timeout = portMAX_DELAY);

    // Flush all remaining blocks to the stepper (decelerates to 0 at end).
    // Call after the last moveTo() to ensure the machine comes to a clean stop.
    bool flush(bool waitForCompletion = false, TickType_t timeout = portMAX_DELAY);

    void waitForIdle();
    bool isFinish();

    ThreeAxisStepper& getStepper() { return dispatcher.getStepper(); } // for testing and advanced use cases

    void stop();
    void resume();

    // Emergency stop — clears planner buffer and resets the stepper.
    void emergency_stop();

    void setCurrentPos(const Vec3& pos);
    Vec3 getCurrentPos();

    // --- Jog interface ---
    // Call setJogAxis() and setJogSpeed() to configure, then call jog() repeatedly
    // (at least once per second) to keep jogging. Stops automatically on timeout.
    void setJogAxis(JogAxis axis) { jogAxis = axis; }
    void setJogSpeed(float speed)  { jogSpeed = fabsf(speed); }
    bool jog(int keepTimeUs = 1000000);// default to 1 second jog timeout

    // Configuration setters (call before motion or between moves)
    void setMaxAcceleration(float accel) { speedPlanner.setMaxAcceleration(accel); }
    void setMaxJerk(float j)             { speedPlanner.setMaxJerk(j); }
    void setJunctionDeviation(float dev) { speedPlanner.setJunctionDeviation(dev); }
    void setMaxSpeed(float feed)      { speedPlanner.setMaxSpeed(feed);}

};