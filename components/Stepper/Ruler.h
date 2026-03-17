#pragma once
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include "mathStructure.h"
#include <esp_attr.h>
class opticsRulerDriver{    // pcnt
private:
    pcnt_unit_handle_t pcnts{NULL};
    bool hasInit{false};
    int offset{0};
    int preCount{0};
    float unit{1.0f};
    float unitInv{1.0f};
public:
    opticsRulerDriver() = default;
    void init(gpio_num_t pinA, gpio_num_t pinB, float unit);
    float getCurrentPos();
    inline int IRAM_ATTR getCounts();
    void setCurrentPos(float newPos);
    void setOrigin();
};

struct ThreeAxisRulerConfig{
    gpio_num_t pinA[3]{GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    gpio_num_t pinB[3]{GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    float unit[3]{0.005f, 0.005f, 0.005f};// in mm
};

class ThreeAxisRuler{
private:
    opticsRulerDriver rulers[3];
public:
    ThreeAxisRuler() = default;
    void init(const ThreeAxisRulerConfig& config);
    Vec3 getCurrentPos();
    void setCurrentPos(const Vec3& newPos);
};
