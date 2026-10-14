#include "control.h"
#include "board_cfg.h"
#include "track.h"
#include "encoder.h"
#include "motor.h"

/* 循迹外环 -> IMU 角速度中环 -> 编码器速度环。 */
float yaw_rate = 0.0f;
uint8 imu_ok = 0;
PID_T pid_speed_left;
PID_T pid_speed_right;
PID_T pid_steer;
PID_T pid_yaw;
SpeedPid_t left;
SpeedPid_t right;
YawParam_t yaw_param;
YawCtrl_t yaw_ctrl;
uint8 pid_running = 0;
uint8 steer_running = 0;
volatile uint8 speed_enable = 0;

float pid_constrain(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void pid_init(PID_T *pid, float kp, float ki, float kd, float target, float limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->limit = limit;
    pid->current = 0.0f;
    pid->out = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->differential = 0.0f;
    pid->integral = 0.0f;
}

void pid_set_target(PID_T *pid, float target)
{
    pid->target = target;
}

void pid_set_limit(PID_T *pid, float limit)
{
    pid->limit = limit;
}

float pid_calculate_positional(PID_T *pid, float current)
{
    pid->current = current;
    pid->error = pid->target - current;
    pid->integral += pid->error;
    pid->p_out = pid->kp * pid->error;
    pid->i_out = pid->ki * pid->integral;
    pid->d_out = pid->kd * (pid->error - pid->last_error);
    pid->out = pid_constrain(pid->p_out + pid->i_out + pid->d_out,
                             -pid->limit, pid->limit);
    pid->last_error = pid->error;
    return pid->out;
}

void lpf_1(float hz, float time, float input, float *output)
{
    float alpha;
    if (hz <= 0.0f || time <= 0.0f)
    {
        *output = input;
        return;
    }
    alpha = 1.0f / (1.0f + 1.0f / (hz * 6.28f * time));
    *output += alpha * (input - *output);
}

/* 速度环采用前馈、误差 PI 和反馈微分。 */
float speed_pid_calculate(PID_T *pid, float current, SpeedPid_t *cfg)
{
    float limited_error = pid_constrain(pid->target - current,
                                        -cfg->err_max, cfg->err_max);
    float feedback_delta;

    pid->current = current;
    pid->error = pid->target - current;
    pid->integral = pid_constrain(pid->integral + limited_error,
                                  -cfg->i_max, cfg->i_max);
    feedback_delta = current - pid->last_feedback;
    lpf_1(cfg->d_hz, cfg->dt,
           feedback_delta, &pid->differential);

    pid->p_out = cfg->kp * pid->error;
    pid->i_out = cfg->ki * pid->integral;
    pid->d_out = -cfg->kd * pid->differential;
    pid->out = cfg->kf * pid->target + pid->p_out + pid->i_out + pid->d_out;
    pid->out = pid_constrain(pid->out, cfg->out_min, cfg->out_max);
    pid->last_feedback = current;
    pid->last_error = pid->error;
    return pid->out;
}

void pid_init_all(void)
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    left.kf = 6.0f;
    left.kp = 6.2f;
    left.ki = 1.0f;
    left.kd = 0.0f;
    left.i_max = 1500.0f;
    left.err_max = 2000.0f;
    left.d_hz = 10.0f;
    left.dt = 0.005f;
    left.out_min = -(float)MOTOR_DUTY_MAX;
    left.out_max = (float)MOTOR_DUTY_MAX;

    /* 左右轮参数独立，便于分别补偿电机和机械差异。 */
    right.kf = 6.0f;
    right.kp = 6.2f;
    right.ki = 1.0f;
    right.kd = 0.0f;
    right.i_max = 1500.0f;
    right.err_max = 2000.0f;
    right.d_hz = 10.0f;
    right.dt = 0.005f;
    right.out_min = -(float)MOTOR_DUTY_MAX;
    right.out_max = (float)MOTOR_DUTY_MAX;

    pid_init(&pid_speed_left, left.kp, left.ki, left.kd, 0.0f, left.out_max);
    pid_init(&pid_speed_right, right.kp, right.ki, right.kd, 0.0f, right.out_max);
    pid_running = 1;
}

/* 角速度环参数：与上面的速度环参数集中放置，便于调参。 */
void yaw_angle_init(void)
{
    yaw_param.kp = 20.8f;
    yaw_param.ki = 0.0f;
    yaw_param.kd = 0.0f;
    yaw_param.out_max = 280.0f;
    yaw_param.filter_alpha = 0.95f;
    pid_init(&pid_yaw, yaw_param.kp, yaw_param.ki, yaw_param.kd,
             0.0f, yaw_param.out_max);
    yaw_ctrl.base_speed = 0.0f;
    yaw_ctrl.diff_speed = 0.0f;
    yaw_ctrl.out_max = yaw_param.out_max;
    yaw_ctrl.running = 0;
}

/* 寻迹环参数：与速度环、角速度环参数集中放置。 */
void steer_init(void)
{
    pid_init(&pid_steer, 40.0f, 0.0f, 0.0f, 0.0f, 280.0f);
    steer_running = 1;
}

void speed_set(float l, float r)
{
    pid_set_target(&pid_speed_left, l);
    pid_set_target(&pid_speed_right, r);
}

void speed_start(void)
{
    bit irq_state;

    if (!imu_ok || !pid_running || !steer_running) return;
    irq_state = EA;
    EA = 0;
    /* 启动先直行，下次灰度采样后由寻迹环更新目标角速度。 */
    yaw_angle_set(0.0f, BASE_SPEED, yaw_param.out_max);
    speed_enable = 1;
    EA = irq_state;
}

void speed_stop(void)
{
    bit irq_state;

    irq_state = EA;
    EA = 0;
    speed_enable = 0;
    yaw_ctrl.running = 0;
    yaw_ctrl.target_yaw_rate = 0.0f;
    yaw_ctrl.base_speed = 0.0f;
    yaw_ctrl.diff_speed = 0.0f;
    motor_stop();
    /* 停机清除积分和上次误差，避免再次启动沿用旧纠偏量。 */
    pid_init(&pid_steer, pid_steer.kp, pid_steer.ki, pid_steer.kd,
             0.0f, pid_steer.limit);
    pid_init(&pid_yaw, yaw_param.kp, yaw_param.ki, yaw_param.kd,
             0.0f, yaw_param.out_max);
    pid_init(&pid_speed_left, left.kp, left.ki, left.kd, 0.0f, left.out_max);
    pid_init(&pid_speed_right, right.kp, right.ki, right.kd, 0.0f, right.out_max);
    EA = irq_state;
}

/* 非零控制量补上电机死区，零输出仍保持停机。 */
float duty_comp(float duty, float deadzone)
{
    if (duty > 0.0f)
    {
        duty += deadzone;
    }
    else if (duty < 0.0f)
    {
        duty -= deadzone;
    }
    return pid_constrain(duty, -(float)MOTOR_DUTY_MAX,
                         (float)MOTOR_DUTY_MAX);
}

void speed_pid_task(void)
{
    float l_out;
    float r_out;
    if (!pid_running) return;
    l_out = speed_pid_calculate(&pid_speed_left, encoder_left.speed, &left);
    r_out = speed_pid_calculate(&pid_speed_right, encoder_right.speed, &right);
#if DEAD_COMP_ENABLE
    l_out = duty_comp(l_out, (float)DEAD_LEFT);
    r_out = duty_comp(r_out, (float)DEAD_RIGHT);
#endif
    /* 屏幕显示补偿后的实际 PWM 指令。 */
    pid_speed_left.out = l_out;
    pid_speed_right.out = r_out;
    motor_set((int16)l_out, (int16)r_out);
}

void pid_wave_task(void)
{
    bit irq_state;
    float target;
    float l_speed;
    float r_speed;
    char target_text[16] = {0};
    char left_text[16] = {0};
    char right_text[16] = {0};
    char line[64];
    uint16 len;

    irq_state = EA;
    EA = 0;
    target = pid_speed_left.target;
    l_speed = encoder_left.speed;
    r_speed = encoder_right.speed;
    EA = irq_state;

    func_float_to_str(target_text, target, 2);
    func_float_to_str(left_text, l_speed, 2);
    func_float_to_str(right_text, r_speed, 2);
    /* USB 单窗口曲线：左轮目标、左反馈、右反馈，单位 cm/s。 */
    sprintf(line, "{PID}%s,%s,%s\n", target_text, left_text, right_text);
    len = (uint16)strlen(line);
    debug_send_buffer((const uint8 *)line, len);
}

void yaw_angle_set(float target_yaw_rate, float base_speed, float out_max)
{
    bit irq_state;

    irq_state = EA;
    EA = 0;
    yaw_ctrl.target_yaw_rate = target_yaw_rate;
    yaw_ctrl.base_speed = base_speed;
    yaw_ctrl.out_max = out_max;
    yaw_ctrl.running = 1;
    EA = irq_state;
}

void yaw_angle_task(void)
{
    float raw;
    if (!speed_enable || !yaw_ctrl.running) return;
    pid_set_limit(&pid_yaw, yaw_ctrl.out_max);
    pid_set_target(&pid_yaw, yaw_ctrl.target_yaw_rate);
    raw = pid_calculate_positional(&pid_yaw, yaw_rate);
    yaw_ctrl.diff_speed = raw * yaw_param.filter_alpha
                        + yaw_ctrl.diff_speed * (1.0f - yaw_param.filter_alpha);
    yaw_ctrl.diff_speed = pid_constrain(yaw_ctrl.diff_speed,
                                        -yaw_ctrl.out_max, yaw_ctrl.out_max);
    speed_set(yaw_ctrl.base_speed + yaw_ctrl.diff_speed,
              yaw_ctrl.base_speed - yaw_ctrl.diff_speed);
}

void steer_task(void)
{
    float target;
    if (!speed_enable || !steer_running) return;
#if TRACK_TEST_ENABLE
    /* 三级串级测试：固定给寻迹环一个误差，不使用实际灰度误差。 */
    target = STEER_YAW_RATE_SIGN * pid_calculate_positional(&pid_steer,
                                                             -TRACK_TEST_ERROR);
#else
    if (!track.valid)
    {
        yaw_angle_set(0.0f, BASE_SPEED, yaw_param.out_max);
        return;
    }
    target = STEER_YAW_RATE_SIGN * pid_calculate_positional(&pid_steer,
                                                             -track_error());
#endif
    /* 寻迹输出按 deg/s 计算，角速度环输入统一使用 rad/s。 */
    target = pid_constrain(target, -280.0f, 280.0f) * RAD_PER_DEG;
    yaw_angle_set(target, BASE_SPEED, yaw_param.out_max);
}

void ctrl_init(void)
{
    motor_init();
    encoder_init();
    track_init();
    imu_ok = 1;
    if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE))
    {
        imu_ok = 0;
    }
    pid_init_all();
    yaw_angle_init();
    steer_init();
    speed_enable = 0;
}

void sensor_task(void)
{
    encoder_sample();
    imu660rc_get_gyro();
    yaw_rate = IMU_YAW_RATE_SIGN * imu660rc_gyro_transition(imu660rc_gyro_z)
             * RAD_PER_DEG;
}

void track_task(void)
{
    track_sample();
    /* 主循环先采样再计算外环，目标角速度原子交给定时器内环。 */
    steer_task();
}
void yaw_task(void)   { yaw_angle_task(); }
void speed_task(void)
{
    if (!speed_enable)
    {
        motor_stop();
        return;
    }

    /* 左右目标由角速度环更新，这里只执行轮速闭环。 */
    speed_pid_task();
}
