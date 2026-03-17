#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "string.h"
#include "functional"
#include "Stepper.h"
#include "Planner.h"
#include "PulseGenerator.h"
#include <esp_timer.h>

#define xPulsePin GPIO_NUM_14
#define xDirPin GPIO_NUM_19
#define yPulsePin GPIO_NUM_13
#define yDirPin GPIO_NUM_18
#define zPulsePin GPIO_NUM_12
#define zDirPin GPIO_NUM_21
#define LoopSignalPin GPIO_NUM_27

static const char* TAG = "Test";

// ---------------------------------------------------------------------------
// Helper: log current position
// ---------------------------------------------------------------------------
static void logPos(Planner& planner, const char* label)
{
    Vec3 p = planner.getCurrentPos();
    ESP_LOGI(TAG, "  [%s] pos = (%.3f, %.3f, %.3f)", label, p.x, p.y, p.z);
}

// ---------------------------------------------------------------------------
// Helper: toggle the loop-signal pin so the logic analyser can see each test
// ---------------------------------------------------------------------------
static bool loopLevel = false;
static void pulseLoopPin()
{
    loopLevel = !loopLevel;
    gpio_set_level(LoopSignalPin, loopLevel);
}

// ---------------------------------------------------------------------------
// Helper: check position is close to expected
// ---------------------------------------------------------------------------
static bool posNear(const Vec3& a, const Vec3& b, float tol = 0.05f)
{
    return length(a - b) < tol;
}
static void assertPos(Planner& planner, const Vec3& expected, const char* context)
{
    Vec3 p = planner.getCurrentPos();
    bool ok = posNear(p, expected);
    if (ok)
        ESP_LOGI(TAG, "  PASS [%s] pos=(%.3f,%.3f,%.3f)", context, p.x, p.y, p.z);
    else
        ESP_LOGE(TAG, "  FAIL [%s] pos=(%.3f,%.3f,%.3f) expected=(%.3f,%.3f,%.3f)",
                 context, p.x, p.y, p.z, expected.x, expected.y, expected.z);
}

// ===================================================================
// TEST 1 — Single-axis move
// ===================================================================
static void test_single_axis(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 1: Single-axis move (X only) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    planner.moveTo(Vec3{10,0,0}, 20);   // 10 mm on X at 20 mm/s
    planner.flush(true);
    assertPos(planner, Vec3{10,0,0}, "X=10");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 2 — Multi-axis diagonal move
// ===================================================================
static void test_diagonal(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 2: Diagonal XY move ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    planner.moveTo(Vec3{10,10,0}, 30);  // 45° diagonal
    planner.flush(true);
    assertPos(planner, Vec3{10,10,0}, "XY=10,10");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 3 — 3-axis move
// ===================================================================
static void test_3axis(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 3: 3-axis XYZ move ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    planner.moveTo(Vec3{5,10,15}, 25);
    planner.flush(true);
    assertPos(planner, Vec3{5,10,15}, "XYZ=5,10,15");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 4 — Chained multi-segment path (junction planning)
// ===================================================================
static void test_chained_segments(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 4: Chained segments (junction planning) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Square path — tests 90° junctions
    planner.moveTo(Vec3{20,0,0},  30);
    planner.moveTo(Vec3{20,20,0}, 30);
    planner.moveTo(Vec3{0,20,0},  30);
    planner.moveTo(Vec3{0,0,0},   30);
    planner.flush(true);
    assertPos(planner, Vec3{0,0,0}, "back to origin");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 5 — Many small collinear segments (look-ahead stress)
// ===================================================================
static void test_many_segments(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 5: 50 small collinear segments ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    for (int i = 1; i <= 50; i++) {
        float x = i * 1.0f;               // 1 mm steps along X
        planner.moveTo(Vec3{x,0,0}, 40);
    }
    planner.flush(true);
    assertPos(planner, Vec3{50,0,0}, "X=50 after 50 segs");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 6 — Zig-zag path (rapid direction changes, junction deviation)
// ===================================================================
static void test_zigzag(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 6: Zig-zag path ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    for (int i = 0; i < 10; i++) {
        float x = (i + 1) * 5.0f;
        float y = (i % 2 == 0) ? 5.0f : 0.0f;
        planner.moveTo(Vec3{x, y, 0}, 35);
    }
    planner.flush(true);
    assertPos(planner, Vec3{50, 0, 0}, "end of zigzag");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 7 — Callbacks between moves
// ===================================================================
static void test_callbacks(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 7: Callbacks between moves ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    int cbCount = 0;

    planner.moveTo(Vec3{5,0,0}, 20);
    planner.addCallback([&planner, &cbCount](){
        cbCount++;
        Vec3 p = planner.getCurrentPos();
        ESP_LOGI("CB", "  callback #1 fired at pos=(%.3f,%.3f,%.3f)", p.x, p.y, p.z);
    });

    planner.moveTo(Vec3{10,0,0}, 20);
    planner.addCallback([&planner, &cbCount](){
        cbCount++;
        Vec3 p = planner.getCurrentPos();
        ESP_LOGI("CB", "  callback #2 fired at pos=(%.3f,%.3f,%.3f)", p.x, p.y, p.z);
    });

    planner.moveTo(Vec3{15,0,0}, 20);
    planner.addCallback([&planner, &cbCount](){
        cbCount++;
        Vec3 p = planner.getCurrentPos();
        ESP_LOGI("CB", "  callback #3 fired at pos=(%.3f,%.3f,%.3f)", p.x, p.y, p.z);
    });

    planner.flush(true);
    assertPos(planner, Vec3{15,0,0}, "X=15 after callbacks");
    ESP_LOGI(TAG, "  callbacks fired: %d (expected 3)", cbCount);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 8 — Negative direction / return to origin
// ===================================================================
static void test_negative_direction(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 8: Negative direction ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    planner.moveTo(Vec3{10,10,10}, 25);
    planner.moveTo(Vec3{0,0,0},    25);  // reverse all axes
    planner.flush(true);
    assertPos(planner, Vec3{0,0,0}, "back to 0,0,0");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 9 — Speed changes between segments
// ===================================================================
static void test_speed_changes(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 9: Speed changes between segments ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    planner.moveTo(Vec3{10,0,0}, 5);   // slow
    planner.moveTo(Vec3{20,0,0}, 30);  // fast
    planner.moveTo(Vec3{30,0,0}, 10);  // slow again
    planner.moveTo(Vec3{40,0,0}, 40);  // max speed
    planner.flush(true);
    assertPos(planner, Vec3{40,0,0}, "X=40 speed changes");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 10 — setCurrentPos (coordinate reset while idle)
// ===================================================================
static void test_setCurrentPos(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 10: setCurrentPos ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{100,200,300});
    assertPos(planner, Vec3{100,200,300}, "after setCurrentPos");

    planner.moveTo(Vec3{110,200,300}, 20);  // move 10mm in X from new origin
    planner.flush(true);
    assertPos(planner, Vec3{110,200,300}, "X=110 from new origin");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 11 — Stop / Resume
// ===================================================================
static void test_stop_resume(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 11: Stop & Resume ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Queue many segments so that some remain in moveQueue when we stop.
    // stop() only decelerates using blocks already in the speed planner;
    // remaining commands in moveQueue are preserved for resume().
    for (int i = 1; i <= 20; i++) {
        planner.moveTo(Vec3{(float)i * 5.0f, 0, 0}, 20);
    }

    // Let motion begin then stop mid-travel
    vTaskDelay(pdMS_TO_TICKS(800));
    Vec3 posBeforeStop = planner.getCurrentPos();
    ESP_LOGI(TAG, "  pos before stop: (%.3f,%.3f,%.3f)",
             posBeforeStop.x, posBeforeStop.y, posBeforeStop.z);

    planner.stop();
    planner.waitForIdle();  // blocks until STOPPED state
    Vec3 posAfterStop = planner.getCurrentPos();
    ESP_LOGI(TAG, "  pos after stop:  (%.3f,%.3f,%.3f)",
             posAfterStop.x, posAfterStop.y, posAfterStop.z);

    // Verify the machine stopped short of the final target
    bool stoppedShort = posAfterStop.x < 95.0f;
    if (stoppedShort)
        ESP_LOGI(TAG, "  PASS stopped short of target (X=%.3f < 95)", posAfterStop.x);
    else
        ESP_LOGE(TAG, "  FAIL did not stop short (X=%.3f)", posAfterStop.x);

    // Resume — the planner returns to NORMAL and keeps consuming moveQueue.
    // Remaining segments that were never pulled should continue executing.
    ESP_LOGI(TAG, "  resuming...");
    planner.resume();

    // After resume add a flush so everything completes
    planner.flush(true);
    Vec3 posAfterResume = planner.getCurrentPos();
    ESP_LOGI(TAG, "  pos after resume+flush: (%.3f,%.3f,%.3f)",
             posAfterResume.x, posAfterResume.y, posAfterResume.z);

    // The final position should be at or past where we stopped (remaining
    // moveQueue segments executed).  It won't necessarily reach 100 because
    // blocks already dispatched during stop are consumed.
    bool resumed = posAfterResume.x > posAfterStop.x + 0.1f;
    if (resumed)
        ESP_LOGI(TAG, "  PASS resumed and moved further (%.3f -> %.3f)",
                 posAfterStop.x, posAfterResume.x);
    else
        ESP_LOGE(TAG, "  FAIL did not advance after resume");

    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 12 — Emergency stop
// ===================================================================
static void test_emergency_stop(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 12: Emergency stop ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    Vec3 posBeforeEStop = planner.getCurrentPos();
    ESP_LOGI(TAG, "  pos before e-stop: (%.3f,%.3f,%.3f)", posBeforeEStop.x, posBeforeEStop.y, posBeforeEStop.z);

    // Queue a long move and let it start
    planner.moveTo(Vec3{100,0,0}, 20);
    planner.flush(false);  // don't wait for completion, we want to e-stop mid-move
    vTaskDelay(pdMS_TO_TICKS(300));

    pulseLoopPin(); // toggle loop pin to mark when e-stop happened
    planner.emergency_stop();

    Vec3 posAfterEStop = planner.getCurrentPos();
    ESP_LOGI(TAG, "  pos after e-stop:  (%.3f,%.3f,%.3f)", posAfterEStop.x, posAfterEStop.y, posAfterEStop.z);

    // After e-stop: planner is idle, position should be preserved (not zeroed)
    bool eStopOk = posAfterEStop.x > 0.0f && posAfterEStop.x < 100.0f;
    if (eStopOk)
        ESP_LOGI(TAG, "  PASS e-stop preserved mid-travel position");
    else
        ESP_LOGE(TAG, "  FAIL e-stop position unexpected");

    // Verify we can move again after e-stop
    Vec3 resumeFrom = posAfterEStop;
    planner.setCurrentPos(resumeFrom);
    planner.moveTo(resumeFrom + Vec3{5,0,0}, 20);
    planner.flush(true);
    assertPos(planner, resumeFrom + Vec3{5,0,0}, "move after e-stop");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 13 — Reset clears everything
// ===================================================================
static void test_reset(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 13: Reset ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Queue several moves then immediately reset
    planner.moveTo(Vec3{10,0,0}, 20);
    planner.moveTo(Vec3{20,0,0}, 20);
    planner.moveTo(Vec3{30,0,0}, 20);
    vTaskDelay(pdMS_TO_TICKS(100));

    planner.reset();
    logPos(planner, "after reset");

    // After reset, position is whatever the stepper had
    // Now set position and verify a clean move works
    planner.setCurrentPos(Vec3{0,0,0});
    planner.moveTo(Vec3{5,0,0}, 20);
    planner.flush(true);
    assertPos(planner, Vec3{5,0,0}, "clean move after reset");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 14 — Buffer full / back-pressure (stress test)
// ===================================================================
static void test_buffer_backpressure(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 14: Buffer back-pressure (120 segments) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    int64_t t0 = esp_timer_get_time();
    for (int i = 1; i <= 120; i++) {
        float x = i * 0.5f;   // 0.5 mm steps
        planner.moveTo(Vec3{x,0,0}, 45);
    }
    planner.flush(true);
    int64_t t1 = esp_timer_get_time();

    assertPos(planner, Vec3{60,0,0}, "X=60 after 120 segs");
    ESP_LOGI(TAG, "  total time: %lld ms", (t1 - t0) / 1000);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 15 — Tiny moves (sub-step resolution)
// ===================================================================
static void test_tiny_moves(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 15: Tiny moves (0.01 mm each) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    for (int i = 1; i <= 20; i++) {
        float x = i * 0.01f;  // 0.01 mm = 2 steps at 0.005 mm/step
        planner.moveTo(Vec3{x,0,0}, 5);
    }
    planner.flush(true);
    assertPos(planner, Vec3{0.2f,0,0}, "X=0.2 after tiny moves");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 16 — Arc approximation (circle via short segments)
// ===================================================================
static void test_circle_approximation(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 16: Circle approximation (72 segments) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    const float radius = 10.0f;
    const int   segments = 72;  // 5° per segment
    // Start at (radius, 0) relative to circle center at origin
    // But we begin at (0,0), so offset center to (-radius, 0) to start at origin
    // Actually let's just trace a circle and come back
    float cx = 0, cy = 0;            // circle center
    float startX = radius, startY = 0;
    planner.setCurrentPos(Vec3{startX, startY, 0});

    for (int i = 1; i <= segments; i++) {
        float angle = (2.0f * M_PI * i) / segments;
        float x = cx + radius * cosf(angle);
        float y = cy + radius * sinf(angle);
        planner.moveTo(Vec3{x, y, 0}, 30);
    }
    planner.flush(true);
    assertPos(planner, Vec3{radius, 0, 0}, "circle end ≈ start");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 17 — Long sustained move (single segment, ~300 mm)
// ===================================================================
static void test_long_move(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 17: Long single move (300 mm) ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    int64_t t0 = esp_timer_get_time();
    planner.moveTo(Vec3{300,0,0}, 20);  // 300 mm at 20 speed
    planner.flush(true);
    int64_t t1 = esp_timer_get_time();

    assertPos(planner, Vec3{300,0,0}, "X=300");
    ESP_LOGI(TAG, "  move time: %lld ms (expected ~15000+ ms for accel/decel)",
             (t1 - t0) / 1000);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 18 — Multiple flush calls (batch-then-flush pattern)
// ===================================================================
static void test_multiple_flushes(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 18: Multiple flush calls ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Batch 1
    planner.moveTo(Vec3{5,0,0}, 20);
    planner.moveTo(Vec3{10,0,0}, 20);
    planner.flush(true);
    assertPos(planner, Vec3{10,0,0}, "after flush 1");

    // Batch 2 — continues from previous position
    planner.moveTo(Vec3{10,5,0}, 20);
    planner.moveTo(Vec3{10,10,0}, 20);
    planner.flush(true);
    assertPos(planner, Vec3{10,10,0}, "after flush 2");

    // Batch 3
    planner.moveTo(Vec3{0,0,0}, 30);
    planner.flush(true);
    assertPos(planner, Vec3{0,0,0}, "after flush 3 — origin");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 19 — Mixed axis moves with callbacks
// ===================================================================
static void test_mixed_axes_with_callbacks(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 19: Mixed axes + callbacks ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    int cbIdx = 0;
    auto makeCB = [&planner, &cbIdx](const char* seg) {
        return [&planner, &cbIdx, seg]() {
            cbIdx++;
            Vec3 p = planner.getCurrentPos();
            ESP_LOGI("CB", "  [%s] cb#%d pos=(%.3f,%.3f,%.3f)", seg, cbIdx, p.x, p.y, p.z);
        };
    };

    planner.moveTo(Vec3{10, 0, 0}, 25);   // X only
    planner.addCallback(makeCB("X"));
    planner.moveTo(Vec3{10,10, 0}, 25);   // Y only
    planner.addCallback(makeCB("Y"));
    planner.moveTo(Vec3{10,10,10}, 25);   // Z only
    planner.addCallback(makeCB("Z"));
    planner.moveTo(Vec3{ 0, 0, 0}, 30);   // diagonal back
    planner.addCallback(makeCB("home"));

    planner.flush(true);
    assertPos(planner, Vec3{0,0,0}, "origin after mixed");
    ESP_LOGI(TAG, "  callbacks fired: %d (expected 4)", cbIdx);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 20 — Parameter changes between batches
// ===================================================================
static void test_parameter_changes(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 20: Parameter changes between batches ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Batch 1: conservative params
    planner.setMaxAcceleration(200);
    planner.setMaxJerk(2000);
    planner.setMaxSpeed(20);
    int64_t t0 = esp_timer_get_time();
    planner.moveTo(Vec3{20,0,0}, 20);
    planner.flush(true);
    int64_t t1 = esp_timer_get_time();
    assertPos(planner, Vec3{20,0,0}, "conservative");
    ESP_LOGI(TAG, "  conservative time: %lld ms", (t1 - t0) / 1000);

    // Batch 2: aggressive params
    planner.setMaxAcceleration(1000);
    planner.setMaxJerk(10000);
    planner.setMaxSpeed(40);
    t0 = esp_timer_get_time();
    planner.moveTo(Vec3{40,0,0}, 40);
    planner.flush(true);
    t1 = esp_timer_get_time();
    assertPos(planner, Vec3{40,0,0}, "aggressive");
    ESP_LOGI(TAG, "  aggressive time: %lld ms", (t1 - t0) / 1000);

    // Restore defaults
    planner.setMaxAcceleration(500);
    planner.setMaxJerk(5000);
    //planner.setMaxAcceleration(100);
    //planner.setMaxJerk(1000);
    planner.setMaxSpeed(40);
    planner.setJunctionDeviation(0.05f);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 21 — Test callback timing
// ===================================================================
static void test_callback_timing(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 21: Callback timing ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    int64_t t0 = esp_timer_get_time();
    planner.moveTo(Vec3{10,0,0}, 20);
    planner.addCallback([&planner, t0](){
        int64_t t = esp_timer_get_time();
        Vec3 p = planner.getCurrentPos();
        ESP_LOGI("CB", "  callback fired at t=%lld ms pos=(%.3f,%.3f,%.3f)",
                 (t - t0) / 1000, p.x, p.y, p.z);
    });
    planner.moveTo(Vec3{20,0,0}, 20);
    planner.flush(true);
    assertPos(planner, Vec3{20,0,0}, "X=20 with callback");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ---------------------------------------------------------------------------
// Test 22: Jog — single-axis continuous jog with timeout auto-stop
// ---------------------------------------------------------------------------
static void test_jog(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 22: Jog ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Jog +X at 20 mm/s for ~3 seconds (call jog() every 200ms)
    planner.setJogAxis(JOG_X_POS);
    planner.setJogSpeed(20.0f);

    ESP_LOGI(TAG, "  Starting jog +X at 20 mm/s...");
    int jogCalls = 15; // 15 * 200ms = 3s of jogging
    for(int i = 0; i < jogCalls; i++){
        bool ok = planner.jog();
        if(i == 0 && !ok){
            ESP_LOGE(TAG, "  FAIL: jog() returned false on first call");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "  Stopped calling jog(), waiting for timeout decel...");
    logPos(planner, "during-decel");

    // Wait for jog to finish (timeout + decel)
    planner.waitForIdle();
    Vec3 finalPos = planner.getCurrentPos();
    ESP_LOGI(TAG, "  Final pos = (%.3f, %.3f, %.3f)", finalPos.x, finalPos.y, finalPos.z);

    // X should have moved roughly (3s jog + 1s timeout) * 20mm/s = ~80mm + accel/decel transients
    if(finalPos.x < 60.0f || finalPos.x > 100.0f){
        ESP_LOGE(TAG, "  FAIL: X position %.3f outside expected range [60, 100]", finalPos.x);
    } else {
        ESP_LOGI(TAG, "  PASS: X=%.3f in expected range", finalPos.x);
    }
    // Y and Z should be ~0
    if(fabsf(finalPos.y) > 0.1f || fabsf(finalPos.z) > 0.1f){
        ESP_LOGE(TAG, "  FAIL: Y=%.3f Z=%.3f should be ~0", finalPos.y, finalPos.z);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ---------------------------------------------------------------------------
// Test 23: Jog — single-axis jog stop
// ---------------------------------------------------------------------------
static void test_jog_stop(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 23: Jog Stop ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Jog +X at 20 mm/s for ~3 seconds (call jog() every 200ms)
    planner.setJogAxis(JOG_X_POS);
    planner.setJogSpeed(20.0f);
    ESP_LOGI(TAG, "  Starting jog +X at 20 mm/s...");
    planner.jog(2000000); // 2 second jog duration, ie. ~3 seconds total with decel
    vTaskDelay(pdMS_TO_TICKS(1000));
    planner.stop();// stop jog mid-move, should decelerate to stop from current speed
    ESP_LOGI(TAG, "  calling stop() mid-way, waiting for timeout decel...");
    logPos(planner, "during-decel");

    // Wait for jog to finish (timeout + decel)
    planner.waitForIdle();
    Vec3 finalPos = planner.getCurrentPos();
    ESP_LOGI(TAG, "  Final pos = (%.3f, %.3f, %.3f)", finalPos.x, finalPos.y, finalPos.z);

    // X should have moved roughly (1s jog + 0.1s segmentTime) * 20mm/s = ~22mm + accel/decel transients
    if(finalPos.x < 20.0f || finalPos.x > 30.0f){
        ESP_LOGE(TAG, "  FAIL: X position %.3f outside expected range [20, 30]", finalPos.x);
    } else {
        ESP_LOGI(TAG, "  PASS: X=%.3f in expected range", finalPos.x);
    }
    // Y and Z should be ~0
    if(fabsf(finalPos.y) > 0.1f || fabsf(finalPos.z) > 0.1f){
        ESP_LOGE(TAG, "  FAIL: Y=%.3f Z=%.3f should be ~0", finalPos.y, finalPos.z);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}


// ---------------------------------------------------------------------------
// Test 24: Jog — low speed jog
// ---------------------------------------------------------------------------
static void test_jog_low_speed(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 24: Jog Low Speed ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Jog +X at 0.5 mm/s for ~3 seconds (call jog() every 200ms)
    planner.setJogAxis(JOG_X_POS);
    planner.setJogSpeed(0.5f);

    ESP_LOGI(TAG, "  Starting jog +X at 0.5 mm/s...");
    int jogCalls = 15; // 15 * 200ms = 3s of jogging
    for(int i = 0; i < jogCalls; i++){
        bool ok = planner.jog();
        if(i == 0 && !ok){
            ESP_LOGE(TAG, "  FAIL: jog() returned false on first call");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "  Stopped calling jog(), waiting for timeout decel...");
    logPos(planner, "during-decel");

    // Wait for jog to finish (timeout + decel)
    planner.waitForIdle();
    Vec3 finalPos = planner.getCurrentPos();
    ESP_LOGI(TAG, "  Final pos = (%.3f, %.3f, %.3f)", finalPos.x, finalPos.y, finalPos.z);

    // X should have moved roughly (3s jog + 1s timeout) * 0.5mm/s = ~2mm + accel/decel transients
    if(finalPos.x < 1.0f || finalPos.x > 3.0f){
        ESP_LOGE(TAG, "  FAIL: X position %.3f outside expected range [1, 3]", finalPos.x);
    } else {
        ESP_LOGI(TAG, "  PASS: X=%.3f in expected range", finalPos.x);
    }
    // Y and Z should be ~0
    if(fabsf(finalPos.y) > 0.1f || fabsf(finalPos.z) > 0.1f){
        ESP_LOGE(TAG, "  FAIL: Y=%.3f Z=%.3f should be ~0", finalPos.y, finalPos.z);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}


// ---------------------------------------------------------------------------
// Test 25: Stable three axis move
// ---------------------------------------------------------------------------
static void test_stable_three_axis_move(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 25: Stable Three Axis Move ======");
    pulseLoopPin();
    planner.reset();

    int count = 0;
    FixedFunction<void()> callback = [&](){
        count++;
        Vec3 p = planner.getCurrentPos();
        ESP_LOGI("CB", "  callback #%d pos=(%.3f,%.3f,%.3f)", count, p.x, p.y, p.z);
    };
    for(int i=0;i<10;i++){
        planner.setCurrentPos(Vec3{-48.71,-48.71,48.65});
        planner.moveTo(Vec3{0,0,0},20);
        planner.addCallback(callback);
        planner.flush(true);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===================================================================
// TEST 26 — Wait between moves
// ===================================================================
static void test_wait(Planner& planner)
{
    ESP_LOGI(TAG, "====== TEST 26: Wait between moves ======");
    pulseLoopPin();
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});

    // Move → wait 1s → move. Verify position and timing.
    int64_t t0 = esp_timer_get_time();

    planner.moveTo(Vec3{10,0,0}, 20);
    planner.addCallback([t0](){
        int64_t t = esp_timer_get_time();
        ESP_LOGI("CB", "  callback fired at t=%lld ms during wait", (t - t0) / 1000);
    });
    planner.addWait(1.0f);   // dwell 1 second at (10,0,0)
    planner.addCallback([t0](){
        int64_t t = esp_timer_get_time();
        ESP_LOGI("CB", "  callback fired at t=%lld ms after wait", (t - t0) / 1000);
    });
    planner.moveTo(Vec3{20,0,0}, 20);
    planner.flush(true);

    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    assertPos(planner, Vec3{20,0,0}, "X=20 after wait");

    // Total time should include the 1s wait plus the two move durations.
    // Each 10 mm move at 20 mm/s (with accel/decel) takes ≥ 500 ms,
    // so total should be well above 1000 ms (the wait alone).
    if(elapsed_ms >= 1000)
        ESP_LOGI(TAG, "  PASS elapsed %lld ms >= 1000 (includes 1s wait)", elapsed_ms);
    else
        ESP_LOGE(TAG, "  FAIL elapsed %lld ms < 1000 (wait may not have worked)", elapsed_ms);

    // --- Wait at the start (from rest) ---
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});
    t0 = esp_timer_get_time();
    planner.addWait(0.5f);
    planner.moveTo(Vec3{5,0,0}, 20);
    planner.flush(true);
    elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    assertPos(planner, Vec3{5,0,0}, "X=5 after initial wait");
    if(elapsed_ms >= 500)
        ESP_LOGI(TAG, "  PASS initial-wait elapsed %lld ms >= 500", elapsed_ms);
    else
        ESP_LOGE(TAG, "  FAIL initial-wait elapsed %lld ms < 500", elapsed_ms);

    // --- Wait at the end (decel to 0, then dwell) ---
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});
    t0 = esp_timer_get_time();
    planner.moveTo(Vec3{5,0,0}, 20);
    planner.addWait(0.5f);
    planner.flush(true);
    elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    assertPos(planner, Vec3{5,0,0}, "X=5 after trailing wait");
    if(elapsed_ms >= 500)
        ESP_LOGI(TAG, "  PASS trailing-wait elapsed %lld ms >= 500", elapsed_ms);
    else
        ESP_LOGE(TAG, "  FAIL trailing-wait elapsed %lld ms < 500", elapsed_ms);

    // --- Multiple waits in sequence ---
    planner.reset();
    planner.setCurrentPos(Vec3{0,0,0});
    t0 = esp_timer_get_time();
    planner.moveTo(Vec3{5,0,0}, 20);
    planner.addWait(0.3f);
    planner.moveTo(Vec3{10,0,0}, 20);
    planner.addWait(0.3f);
    planner.moveTo(Vec3{15,0,0}, 20);
    planner.flush(true);
    elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    assertPos(planner, Vec3{15,0,0}, "X=15 after multi-wait");
    if(elapsed_ms >= 600)
        ESP_LOGI(TAG, "  PASS multi-wait elapsed %lld ms >= 600", elapsed_ms);
    else
        ESP_LOGE(TAG, "  FAIL multi-wait elapsed %lld ms < 600", elapsed_ms);

    vTaskDelay(pdMS_TO_TICKS(500));
}

// ===========================================================================
extern "C" void app_main(void)
{
    // --- GPIO setup ---
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    io_conf.pin_bit_mask = (1ULL << LoopSignalPin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(LoopSignalPin, 0));

    // --- Planner init ---
    StepperConfig config = {
        .PulsePin  = {xPulsePin, yPulsePin, zPulsePin},
        .DirPin    = {xDirPin,   yDirPin,   zDirPin},
        .StepUnit  = {0.005f, 0.005f, 0.005f}
    };
    Planner planner;
    planner.init(config);
    planner.setMaxAcceleration(100);   // mm/s²
    planner.setMaxJerk(1000);          // mm/s³
    planner.setJunctionDeviation(0.05f);
    planner.setMaxSpeed(40);           // mm/s

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   PLANNER COMPREHENSIVE TEST SUITE");
    ESP_LOGI(TAG, "==========================================");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run all tests in order — each one resets the planner
    test_single_axis(planner);            // 1
    test_diagonal(planner);               // 2
    test_3axis(planner);                  // 3
    test_chained_segments(planner);       // 4
    test_many_segments(planner);          // 5
    test_zigzag(planner);                 // 6
    test_callbacks(planner);              // 7
    test_negative_direction(planner);     // 8
    test_speed_changes(planner);          // 9
    test_setCurrentPos(planner);          // 10
    test_stop_resume(planner);            // 11
    test_emergency_stop(planner);         // 12
    test_reset(planner);                  // 13
    test_buffer_backpressure(planner);    // 14
    test_tiny_moves(planner);             // 15
    test_circle_approximation(planner);   // 16
    test_long_move(planner);              // 17
    test_multiple_flushes(planner);       // 18
    test_mixed_axes_with_callbacks(planner); // 19 
    test_parameter_changes(planner);      // 20
    test_callback_timing(planner);       // 21
    test_jog(planner);                    // 22
    test_jog_stop(planner);               // 23
    test_wait(planner);                   // 26

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   ALL TESTS COMPLETE");
    ESP_LOGI(TAG, "==========================================");

    // Keep alive — repeat all tests in a loop for long-running soak
    int iteration = 1;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "=== Soak iteration %d ===", ++iteration);
        
        test_single_axis(planner);            // 1
        test_diagonal(planner);               // 2
        test_3axis(planner);                  // 3
        test_chained_segments(planner);       // 4
        test_many_segments(planner);          // 5
        test_zigzag(planner);                 // 6
        test_callbacks(planner);              // 7
        test_negative_direction(planner);     // 8
        test_speed_changes(planner);          // 9
        test_setCurrentPos(planner);          // 10
        test_stop_resume(planner);            // 11
        test_emergency_stop(planner);         // 12
        test_reset(planner);                  // 13
        test_buffer_backpressure(planner);    // 14
        test_tiny_moves(planner);             // 15
        test_circle_approximation(planner);   // 16
        test_long_move(planner);              // 17
        test_multiple_flushes(planner);       // 18
        test_mixed_axes_with_callbacks(planner); // 19
        test_parameter_changes(planner);      // 20
        test_callback_timing(planner);       // 21
        test_jog(planner);                    // 22
        test_jog_stop(planner);               // 23
        test_wait(planner);                   // 26
        ESP_LOGI(TAG, "=== Soak iteration %d complete ===", iteration);
    }
}
