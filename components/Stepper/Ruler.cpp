#include "Ruler.h"
#include <driver/gpio.h>
#define opticsRulerHighLimit 32000
#define opticsRulerLowLimit -32000

void opticsRulerDriver::init(gpio_num_t pinA, gpio_num_t pinB, float unit_in)
{
    hasInit = true;
    unit = unit_in;
    unitInv = 1.0f / unit_in;
    pcnt_unit_config_t unit_config = {
        .low_limit = opticsRulerLowLimit,
        .high_limit = opticsRulerHighLimit,
        .intr_priority = 2,
        .flags = 1,
    };
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000,
    };
    pcnt_chan_config_t chan_config = {};
    pcnt_unit_handle_t& pcnt_unit = pcnts;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_channel_handle_t pcnt_chan_a = NULL, pcnt_chan_b = NULL;
    chan_config.edge_gpio_num = pinB;
    chan_config.level_gpio_num = pinA;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan_a));
    std::swap(chan_config.edge_gpio_num, chan_config.level_gpio_num);
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, opticsRulerHighLimit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, opticsRulerLowLimit));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

float opticsRulerDriver::getCurrentPos()
{
    if(!hasInit) return 0.0f;
    return getCounts() * unit;
}

int opticsRulerDriver::getCounts()
{
    if(!hasInit) return 0;
    int count;
    if(pcnt_unit_get_count(pcnts,&count) == ESP_OK)
        preCount = count;
    return preCount + offset;
}


void opticsRulerDriver::setCurrentPos(float newPos)
{
    if(!hasInit) return;
    setOrigin();
    offset = newPos * unitInv;
}

void opticsRulerDriver::setOrigin()
{
    if(!hasInit) return;
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnts));
    preCount = 0;
    offset = 0;
}

void ThreeAxisRuler::init(const ThreeAxisRulerConfig &config)
{
    for(int i=0;i<3;i++){
        rulers[i].init(config.pinA[i], config.pinB[i], config.unit[i]);
    }
}

Vec3 ThreeAxisRuler::getCurrentPos()
{
    return {
        rulers[0].getCurrentPos(),
        rulers[1].getCurrentPos(),
        rulers[2].getCurrentPos(),
    };
}

void ThreeAxisRuler::setCurrentPos(const Vec3 &newPos)
{
    rulers[0].setCurrentPos(newPos.x);
    rulers[1].setCurrentPos(newPos.y);
    rulers[2].setCurrentPos(newPos.z);
}
