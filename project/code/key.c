#include "key.h"
#include "board_cfg.h"
#include "control.h"

#define KEY_COUNT           (3U)  /* 按键数量：B1、B2、B3 */
#define KEY_IDLE            (0U)  /* 等待第一次按下 */
#define KEY_HOLD            (1U)  /* 第一次按住 */
#define KEY_WAIT            (2U)  /* 等待第二次按下 */
#define KEY_SECOND          (3U)  /* 第二次按住 */
#define KEY_LONG_HOLD       (4U)  /* 长按已触发 */
#define KEY_DEBOUNCE_COUNT  (3U)  /* 连续 3 次相同电平才确认 */
#define KEY_LONG_COUNT      (300U / KEY_SCAN_MS) /* 300 ms 长按 */
#define KEY_DOUBLE_COUNT    (150U / KEY_SCAN_MS) /* 150 ms 双击间隔 */

key_t keys[KEY_COUNT];
uint8 key_value = 0;
uint8 key_down = 0;
uint8 key_up = 0;
uint8 key_short = 0;
uint8 key_long = 0;
uint8 key_double = 0;

void key_init(void)
{
    uint8 i;

    keys[0].pin = KEY_B1_PIN;
    keys[0].mask = KEY_B1;
    keys[1].pin = KEY_B2_PIN;
    keys[1].mask = KEY_B2;
    keys[2].pin = KEY_B3_PIN;
    keys[2].mask = KEY_B3;

    key_value = 0;
    key_down = 0;
    key_up = 0;
    key_short = 0;
    key_long = 0;
    key_double = 0;

    for (i = 0; i < KEY_COUNT; i++)
    {
        gpio_init(keys[i].pin, GPI, GPIO_HIGH, GPI_PULL_UP);
        keys[i].level = gpio_get_level(keys[i].pin);
        keys[i].debounce = 0;
        keys[i].ticks = 0;
        keys[i].state = KEY_IDLE;
        if (keys[i].level == GPIO_LOW)
        {
            key_value |= keys[i].mask;
        }
    }
}

void key_task(void)
{
    uint8 i;
    uint8 raw;
    uint8 changed;
    key_t *key;

    for (i = 0; i < KEY_COUNT; i++)
    {
        key = &keys[i];
        raw = gpio_get_level(key->pin);
        changed = 0;

        if (raw != key->level)
        {
            if (++key->debounce >= KEY_DEBOUNCE_COUNT)
            {
                key->level = raw;
                key->debounce = 0;
                changed = 1;
                if (raw == GPIO_LOW)
                {
                    key_value |= key->mask;
                }
                else
                {
                    key_value &= (uint8)~key->mask;
                }
            }
        }
        else
        {
            key->debounce = 0;
        }

        switch (key->state)
        {
            case KEY_IDLE:
                if (changed && key->level == GPIO_LOW)
                {
                    key_down |= key->mask;
                    key->ticks = 0;
                    key->state = KEY_HOLD;
                }
                break;

            case KEY_HOLD:
                key->ticks++;
                if (changed && key->level == GPIO_HIGH)
                {
                    key_up |= key->mask;
                    key->ticks = 0;
                    key->state = KEY_WAIT;
                }
                else if (key->ticks >= KEY_LONG_COUNT)
                {
                    key_long |= key->mask;
                    key->state = KEY_LONG_HOLD;
                }
                break;

            case KEY_WAIT:
                key->ticks++;
                if (changed && key->level == GPIO_LOW)
                {
                    key_down |= key->mask;
                    key->ticks = 0;
                    key->state = KEY_SECOND;
                }
                else if (key->ticks >= KEY_DOUBLE_COUNT)
                {
                    key_short |= key->mask;
                    key->state = KEY_IDLE;
                }
                break;

            case KEY_SECOND:
                if (changed && key->level == GPIO_HIGH)
                {
                    key_up |= key->mask;
                    key_double |= key->mask;
                    key->state = KEY_IDLE;
                }
                break;

            case KEY_LONG_HOLD:
                if (changed && key->level == GPIO_HIGH)
                {
                    key_up |= key->mask;
                    key->state = KEY_IDLE;
                }
                break;

            default:
                key->state = KEY_IDLE;
                break;
        }
    }
}

void key_clear(uint8 mask)
{
    key_down &= (uint8)~mask;
    key_up &= (uint8)~mask;
    key_short &= (uint8)~mask;
    key_long &= (uint8)~mask;
    key_double &= (uint8)~mask;
}

/* 按键扫描和启动/停止动作放在按键模块内。 */
void key_proc(void)
{
    key_task();
    if (key_down & KEY_B3)
    {
        key_down &= (uint8)~KEY_B3;
        if (speed_enable)
        {
            speed_stop();
        }
        else
        {
            speed_start();
        }
    }
}
