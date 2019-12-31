#ifndef __WS2818_H
#define __WS2818_H

#include <stdint.h>
#include <string.h>

#define LEDS_CNT 60

enum LED_TYPE {
    LED_SENS_FALLING,
    LED_SENS_RAISING,
    LED_SENS_100,
    LED_SENS_OFF,
    LED_SENS_DEFAULT,
};

#pragma pack(push, 1)
struct LED {
    float __intens;
    float intens_inc;
    float intens_min;
    float intens_max;
    void (*update_time)(void);
    void (*clear_time)(void);
    uint8_t R;
    uint8_t G;
    uint8_t B;
    enum LED_TYPE type;
    uint16_t time_divider;
    uint16_t __time;
    uint8_t pos;
    uint8_t repeate;
};
#pragma pack(pop)

void leds_to_dma(const struct LED *led, const uint8_t length);
void led_buf_to_dma(const uint8_t R, const uint8_t G, const uint8_t B, const uint8_t addr);
void leds_light(void);
void leds_init(void *ins, const uint8_t length);
void leds_clear(void);

#endif //__WS2818_H
