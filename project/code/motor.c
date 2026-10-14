#include "motor.h"
#include "board_cfg.h"

int16 motor_limit(int16 duty)
{
    if (duty > (int16)MOTOR_DUTY_MAX) return (int16)MOTOR_DUTY_MAX;
    if (duty < -(int16)MOTOR_DUTY_MAX) return -(int16)MOTOR_DUTY_MAX;
    return duty;
}

void motor_write_one(gpio_pin_enum dir_pin, pwm_channel_enum pwm_pin, int16 duty)
{
    uint16 value;

    duty = motor_limit(duty);
    if (duty >= 0)
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
        value = (uint16)duty;
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_LOW);
        value = (uint16)(-duty);
    }
    pwm_set_duty(pwm_pin, value);
}

void motor_init(void)
{
    gpio_init(MOTOR_LEFT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQ, 0);
    motor_stop();
}

void motor_set(int16 left, int16 right)
{
    motor_write_one(MOTOR_LEFT_DIR, MOTOR_LEFT_PWM, (int16)(left * MOTOR_LEFT_SIGN));
    motor_write_one(MOTOR_RIGHT_DIR, MOTOR_RIGHT_PWM, (int16)(right * MOTOR_RIGHT_SIGN));
}

void motor_stop(void)
{
    pwm_set_duty(MOTOR_LEFT_PWM, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM, 0);
}
