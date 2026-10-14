#ifndef APP_TRACK_H
#define APP_TRACK_H

#include "zf_common_headfile.h"

#define TRACK_N (16U) /* 龙邱灰度通道数 */

typedef struct
{
    uint16 adc;             /* 最近一次 AS 原始值 */
    uint16 raw[TRACK_N];    /* 16 路 12 位 ADC 原始值 */
    uint8 gray[TRACK_N];    /* 龙邱定义的 0~100 灰度值 */
    float error;            /* 放大后的循迹误差，范围为 ±TRACK_GAIN */
    uint8 valid;            /* 数据有效标志 */
} track_t;

extern track_t track;

void track_init(void);
void track_sample(void);
float track_error(void);

#endif
