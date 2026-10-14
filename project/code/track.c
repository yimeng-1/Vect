#include "track.h"
#include "board_cfg.h"

track_t track;

void track_init(void)
{
    uint8 i;

    gpio_init(TRACK_S0, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(TRACK_S3, GPO, GPIO_LOW, GPO_PUSH_PULL);
#if !DEBUG_WIRELESS
    adc_init(TRACK_AS, ADC_12BIT);
#endif

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
    uint32 adc_sum;
    uint32 raw_sum;
    uint32 gray_sum;
    float pos;

#if DEBUG_WIRELESS
    /* wireless debug disables gray ADC to avoid P04/UART5 RX conflict */
    return;
#endif

    raw_sum = 0;
    gray_sum = 0;
    pos = 0.0f;

    for (i = 0; i < TRACK_N; i++)
    {
        /* S3~S0 按高位到低位组成通道号，数组下标直接对应 D0~D15。 */
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
        track.gray[i] = (uint8)(((uint32)raw * 100U + 2047U) / 4095U);
        track.adc = raw;

        if (TRACK_INV)
        {
            raw = 4095U - raw;
            signal = 100U - track.gray[i];
        }
        else
        {
            signal = track.gray[i];
        }

        raw_sum += raw;
        gray_sum += signal;
        pos += (float)signal * ((float)i - 7.5f);
    }

    track.valid = (raw_sum >= TRACK_SUM_MIN && gray_sum > 0U) ? 1 : 0;
    if (track.valid)
    {
        track.error = pos * TRACK_GAIN / ((float)gray_sum * 7.5f);
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
