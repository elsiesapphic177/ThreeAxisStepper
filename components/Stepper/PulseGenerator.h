#pragma once
#include <cstdint>
#include "driver/gpio.h"
#include "StepperConfig.h"

class PulseGenerator{ // interface
public:
    virtual ~PulseGenerator() = default;
    virtual void addPulse(const int64_t& pulseTime, bool direction) = 0;// pulseTime must > last pulse time, ie. in sequence
    virtual void reset() = 0;
    virtual bool isRunning() const = 0;

    virtual int32_t getCount() const = 0;
    virtual void setCount(int32_t count) = 0;
};

class MCPWMPulseGenerator:public PulseGenerator{
private:
    struct Impl;
    Impl* impl{nullptr};
public:
    virtual ~MCPWMPulseGenerator();
    void init(gpio_num_t targetPin_in, gpio_num_t dirPin_in, bool dirInverse = false);
    // make sure pulseTime is in sequence, not too close and not too far from current time
    void addPulse(const int64_t& pulseTime, bool direction) override;
    void reset() override;
    bool isRunning() const override;

    int32_t getCount() const override;
    void setCount(int32_t count) override;
};