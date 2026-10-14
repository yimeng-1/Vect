#include "encoder.h"
#include "board_cfg.h"

encoder_t encoder_left;
encoder_t encoder_right;

void encoder_init(void)
{
    encoder_left.delta = 0;
    encoder_left.total = 0;
    encoder_left.speed = 0.0f;
    encoder_right.delta = 0;
    encoder_right.total = 0;
    encoder_right.speed = 0.0f;

    /* 144K 库的参数顺序是：定时器、脉冲脚、方向脚。 */
    encoder_dir_init(ENCODER_LEFT_INDEX, ENCODER_LEFT_PULSE, ENCODER_LEFT_DIR);
    encoder_dir_init(ENCODER_RIGHT_INDEX, ENCODER_RIGHT_PULSE, ENCODER_RIGHT_DIR);
}

void encoder_sample(void)
{
    encoder_left.delta = (int16)(encoder_get_count(ENCODER_LEFT_INDEX) * ENCODER_LEFT_SIGN);
    encoder_right.delta = (int16)(encoder_get_count(ENCODER_RIGHT_INDEX) * ENCODER_RIGHT_SIGN);
    encoder_clear_count(ENCODER_LEFT_INDEX);
    encoder_clear_count(ENCODER_RIGHT_INDEX);

    encoder_left.total += encoder_left.delta;
    encoder_right.total += encoder_right.delta;
    encoder_left.speed = (float)encoder_left.delta * ENCODER_LEFT_SPEED_SCALE;
    encoder_right.speed = (float)encoder_right.delta * ENCODER_RIGHT_SPEED_SCALE;
}
