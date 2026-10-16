#include "track.h"
#include "board_cfg.h"
#include "control.h"

track_t track;

void track_init(void)
{
    uint8 i;

    gpio_init(TRACK_S0, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S3, GPO, GPIO_LOW, GPO_PUSH_PULL);
    adc_init(TRACK_AS, ADC_12BIT);

    track.adc = 0;
    for (i = 0; i < TRACK_N; i++)
    {
        track.raw[i] = 0;
        track.gray[i] = 0;
    }
    track.error = 0.0f;
    track.valid = 0;
}

void track_sample(void)
{
    uint8 i;
    uint8 j;
    uint16 raw;
    uint16 signal;
    uint16 low;
    uint16 high;
    uint32 adc_sum;
    uint32 raw_sum;
    uint32 gray_sum;
    float pos;

    raw_sum = 0;
    gray_sum = 0;
    pos = 0.0f;
    low = TRACK_LEVEL_MAX;
    high = 0;

    for (i = TRACK_FIRST; i <= TRACK_LAST; i++)
    {
        /* D0～D15 全部参与采样和循迹。 */
        gpio_set_level(TRACK_S3, (i >> 3) & 0x01);
        gpio_set_level(TRACK_S2, (i >> 2) & 0x01);
        gpio_set_level(TRACK_S1, (i >> 1) & 0x01);
        gpio_set_level(TRACK_S0, i & 0x01);
        system_delay_us(5);

        /* 与龙邱例程一致：采样五次，丢弃前三次，平均后两次。 */
        adc_sum = 0;
        for (j = 0; j < 5U; j++)
        {
            raw = adc_convert(TRACK_AS);
            if (j >= 3U)
            {
                adc_sum += raw;
            }
        }

        raw = (uint16)((adc_sum + 1U) / 2U);
        track.raw[i] = raw;
        track.gray[i] = (uint16)(((uint32)raw * TRACK_LEVEL_MAX + 2047U) / 4095U);
        track.adc = raw;

        if (TRACK_INV)
        {
            raw = 4095U - raw;
            signal = TRACK_LEVEL_MAX - track.gray[i];
        }
        else
        {
            signal = track.gray[i];
        }
        if (signal < low) low = signal;
        if (signal > high) high = signal;

        raw_sum += raw;
        gray_sum += signal;
        pos += (float)signal * ((float)i - TRACK_CENTER);
    }

    /* 均匀底色不作为有效线；此标志仅用于误差有效性判断，不触发停机。 */
    track.valid = (raw_sum >= TRACK_SUM_MIN && gray_sum > 0U
                   && high - low >= TRACK_DIFF_MIN) ? 1 : 0;
    if (track.valid)
    {
        /* 将通道偏移映射到 ±TRACK_ERR_MAX 的误差量程。 */
        track.error = pos * TRACK_ERR_MAX / ((float)gray_sum * TRACK_SPAN);
    }
    else
    {
        track.error = 0.0f;
    }
}

float track_error(void)
{
    return track.valid ? track.error : 0.0f;
}

/* 灰度采样和寻迹外环放在灰度模块内。 */
void track_task(void)
{
    track_sample();
    steer_task();
}
