#ifndef APP_KEY_H
#define APP_KEY_H

#include "zf_common_headfile.h"

#define KEY_B1              (0x01U)  /* 主板按键 B1 */
#define KEY_B2              (0x02U)  /* 主板按键 B2 */
#define KEY_B3              (0x04U)  /* 主板按键 B3 */

typedef struct
{
    gpio_pin_enum pin;  /* GPIO 引脚 */
    uint8 mask;         /* 按键位掩码 */
    uint8 level;        /* 消抖后的电平 */
    uint8 debounce;     /* 消抖计数 */
    uint16 ticks;       /* 状态计时 */
    uint8 state;        /* 按键状态机 */
} key_t;

extern uint8 key_value;
extern uint8 key_down;
extern uint8 key_up;
extern uint8 key_short;
extern uint8 key_long;
extern uint8 key_double;

void key_init(void);
void key_task(void);
void key_proc(void);
void key_clear(uint8 mask);

#endif
