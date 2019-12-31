#include "stm32f1xx_hal.h"

#include "ws2818.h"

#define LEDS_BUF (LEDS_CNT * 24 + LEDS_DELAY)
#define LEDS_DELAY (uint16_t)(50/1.3 + 10)
#define LEDS_HIGH (uint16_t)(htim3.Init.Period / 1.8)
#define LEDS_LOW (uint16_t)(htim3.Init.Period / 3)

extern TIM_HandleTypeDef htim3;

static void _update_time(void);
static void _clear_time(void);

struct LED *m_led;
uint8_t m_length;

void leds_init(void *ins, const uint8_t length)
{
    m_led = ins;
    m_length = length;

    for(uint8_t i = 0; i < length; ++i) {
        m_led[i].update_time = _update_time;
        m_led[i].clear_time = _clear_time;
        m_led[i].R = 0x0;
        m_led[i].G = 0x0;
        m_led[i].B = 0x0;
        m_led[i].__intens = 0.;
        m_led[i].intens_inc = 0.;
        m_led[i].type = LED_SENS_DEFAULT;
        //m_led[i].__time = 0x0;
        m_led[i].pos = i;
        m_led[i].repeate = 0;
        m_led[i].time_divider = 0;
    }
}

static void _clear_time(void)
{
    for(uint8_t i = 0; i < m_length; ++i) {
        m_led[i].clear_time = 0x0;
    }
}

static void _update_time(void)
{
    for(uint8_t i = 0; i < m_length; ++i) {
        struct LED *ptr = &m_led[i];
        switch((uint8_t)ptr->type) {
            case LED_SENS_FALLING: {
                if(ptr->__intens > ptr->intens_min) {
                    if(!ptr->time_divider || !(ptr->__time % ptr->time_divider)) {
                        ptr->__intens -= ptr->intens_inc;
                    }
                } else if(ptr->repeate) {
                    ptr->__intens = ptr->intens_max;
                }
                break;
            }
            case LED_SENS_RAISING: {
                if(ptr->__intens < ptr->intens_max) {
                    if(!ptr->time_divider || !(ptr->__time % ptr->time_divider)) {
                        ptr->__intens += ptr->intens_inc;
                    }
                } else if(ptr->repeate) {
                    ptr->__intens = ptr->intens_min;
                }
                break;
            }
            case LED_SENS_100: {
                ptr->__intens = ptr->intens_max;
                break;
            }
            case LED_SENS_OFF: {
                ptr->__intens = ptr->intens_min;
                break;
            }
            default:
                ;
        }
        ptr->__time++;
    }
}

void leds_to_dma(const struct LED *led, const uint8_t length)
{
    for(uint8_t i = 0; i < length; ++i) {
        const struct LED *ptr = &led[i];
        led_buf_to_dma(ptr->R * ptr->__intens, ptr->G * ptr->__intens, ptr->B * ptr->__intens, ptr->pos);
    }
}

static uint16_t leds_buf[LEDS_BUF + LEDS_DELAY];

void led_buf_to_dma(const uint8_t R, const uint8_t G, const uint8_t B, const uint8_t addr)
{

    for(uint8_t i = 0; i < 8; ++i) {
        const uint16_t addr_r = (i + 8) + (24 * addr) + LEDS_DELAY;
        const uint16_t addr_g = (i + 0) + (24 * addr) + LEDS_DELAY;
        const uint16_t addr_b = (i + 16) + (24 * addr) + LEDS_DELAY;
        if((R >> (7-i)) & 0x1) {
            leds_buf[addr_r] = LEDS_HIGH;
        } else {
            leds_buf[addr_r] = LEDS_LOW;
        }
        if((G >> (7-i)) & 0x1) {
            leds_buf[addr_g] = LEDS_HIGH;
        } else {
            leds_buf[addr_g] = LEDS_LOW;
        }
        if((B >> (7-i)) & 0x1) {
            leds_buf[addr_b] = LEDS_HIGH;
        } else {
            leds_buf[addr_b] = LEDS_LOW;
        }
    }
}

void leds_light()
{
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)&leds_buf, sizeof(leds_buf) / sizeof(uint16_t));
}

void leds_clear()
{
    for(uint8_t j = 0; j < LEDS_CNT; ++j)
        for(uint8_t i = 0; i < 8; ++i)
        {
            leds_buf[LEDS_DELAY + i + 8 + 24 * j] = LEDS_LOW;
            leds_buf[LEDS_DELAY + i + 0 + 24 * j] = LEDS_LOW;
            leds_buf[LEDS_DELAY + i + 16 + 24 * j] = LEDS_LOW;
        }
}
