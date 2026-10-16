#ifndef APP_TRACK_H
#define APP_TRACK_H

#include "zf_common_headfile.h"

#define TRACK_N      (16U) /* 龙邱灰度总通道数 */
#define TRACK_FIRST  (0U)  /* 首个参与循迹的通道 D0 */
#define TRACK_LAST   (15U) /* 最后参与循迹的通道 D15 */
#define TRACK_CENTER (7.5f) /* D0～D15 的几何中心 */
#define TRACK_SPAN   (7.5f) /* 中心到有效边缘的距离 */

typedef struct
{
    uint16 adc;             /* 最近一次 AS 原始值 */
    uint16 raw[TRACK_N];    /* 16 路 12 位 ADC 原始值 */
    uint16 gray[TRACK_N];   /* 归一化灰度值，范围 0～1000 */
    float error;            /* 位置误差，范围约为 ±TRACK_ERR_MAX */
    uint8 valid;            /* 数据有效标志 */
} track_t;

extern track_t track;

void track_init(void);
void track_sample(void);
void track_task(void);
float track_error(void);

#endif
