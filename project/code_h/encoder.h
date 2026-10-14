#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include "zf_common_headfile.h"

typedef struct
{
    int16 delta;  /* 本周期脉冲增量 */
    int32 total;  /* 累计脉冲 */
    float speed;  /* 实际速度，单位 cm/s */
} encoder_t;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

void encoder_init(void);
void encoder_sample(void);

#endif
