#include "control.h"
#include "board_cfg.h"
#include "track.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "scheduler.h"

/* 循迹外环 -> IMU 角速度中环 -> 编码器速度环。 */
PID_T pid_speed_left;
PID_T pid_speed_right;
PID_T pid_steer;
PID_T pid_yaw;
float steer_in = 0.0f;
float steer_out = 0.0f;
SpeedPid_t left;
SpeedPid_t right;
YawParam_t yaw_param;
YawCtrl_t yaw_ctrl;
uint8 pid_running = 0;
uint8 steer_running = 0;
volatile uint8 speed_enable = 0;

typedef struct
{
    float kp;      /* 寻迹环比例系数 */
    float ki;      /* 寻迹环积分系数 */
    float kd;      /* 寻迹环微分系数 */
    float speed;   /* 本阶段基础速度，cm/s */
    float out_max; /* 本阶段目标角速度限幅，deg/s */
} SteerStage_t;

typedef struct
{
    SteerStage_t straight; /* 误差较小，保持高速直行 */
    SteerStage_t curve;    /* 误差中等，降低速度并增强纠偏 */
    SteerStage_t turn;     /* 误差很大，按直角弯处理 */
    uint16 turn_ms;        /* 直角弯转向目标保持时间，ms */
} SteerPlan_t;

/* 误差分段策略：直道、普通弯道、直角弯各用一套参数。 */
SteerPlan_t steer_plan = {
    {4.2f, 0.0f, 2.8f, 200.0f, 650.0f}, /* |误差| <= 30 */
    {4.2f, 0.0f, 2.8f, 200.0f, 650.0f},     /* 普通弯道 */
    {4.2f, 0.0f, 2.8f, 200.0f, 650.0f},    /* |误差| >= 200 */
    200                                  /* 直角弯保持时间，ms */
};
uint8 steer_stage_id = 0;

typedef struct
{
    uint32 start;  /* 开始转弯的毫秒时刻 */
    float target;  /* 触发时的目标角速度，保持方向及大小 */
    uint8 active;  /* 正在保持转向 */
    uint8 ready;   /* 回到直道后允许再次触发，避免同一弯反复锁定 */
} Turn_t;

Turn_t turn;

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
    left.kp = 6.5f;
    left.ki = 0.0f;
    left.kd = 1.0f;
    left.i_max = 1500.0f;
    left.err_max = 2000.0f;
    left.d_hz = 10.0f;
    left.dt = 0.005f;
    left.out_min = -(float)MOTOR_DUTY_MAX;
    left.out_max = (float)MOTOR_DUTY_MAX;

    /* 左右轮参数独立，便于分别补偿电机和机械差异。 */
    right.kf = 6.0f;
    right.kp = 6.5f;
    right.ki = 0.0f;
    right.kd = 1.0f;
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
    yaw_param.kp = 2.75f;
    yaw_param.ki = 0.0f;
    yaw_param.kd = 1.6f;
    yaw_param.out_max = 800.0f;
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
    pid_init(&pid_steer, steer_plan.straight.kp, steer_plan.straight.ki,
             steer_plan.straight.kd, 0.0f, steer_plan.straight.out_max);
    steer_stage_id = 0;
    turn.active = 0;
    turn.ready = 1;
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

    if (!imu_ok || !pid_running) return;
    if (!steer_running) return;
    irq_state = EA;
    EA = 0;
    /* 启动先直行，下次灰度采样后由寻迹环更新目标角速度。 */
    yaw_angle_set(0.0f, steer_plan.straight.speed, yaw_param.out_max);
    speed_enable = 1;
    EA = irq_state;
}

void speed_stop(void)
{
    bit irq_state;

    irq_state = EA;
    EA = 0;
    speed_enable = 0;
    turn.active = 0;
    turn.ready = 1;
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

void speed_pid_task(void)
{
    float l_out;
    float r_out;
    if (!pid_running) return;
    l_out = speed_pid_calculate(&pid_speed_left, encoder_left.speed, &left);
    r_out = speed_pid_calculate(&pid_speed_right, encoder_right.speed, &right);
    /* 屏幕显示最终下发的 PWM 指令。 */
    pid_speed_left.out = l_out;
    pid_speed_right.out = r_out;
    motor_set((int16)l_out, (int16)r_out);
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
    /* 位置式角速度环：目标角速度减 IMU 实际角速度。 */
    raw = pid_calculate_positional(&pid_yaw, yaw_rate);
    raw = pid_constrain(raw, -yaw_ctrl.out_max, yaw_ctrl.out_max);
    yaw_ctrl.diff_speed = raw * yaw_param.filter_alpha
                        + yaw_ctrl.diff_speed * (1.0f - yaw_param.filter_alpha);
    yaw_ctrl.diff_speed = pid_constrain(yaw_ctrl.diff_speed,
                                        -yaw_ctrl.out_max, yaw_ctrl.out_max);
    speed_set(yaw_ctrl.base_speed + yaw_ctrl.diff_speed,
              yaw_ctrl.base_speed - yaw_ctrl.diff_speed);
}

/* 根据误差选择当前链式控制阶段，返回值对应 steer_plan 的阶段。 */
uint8 steer_stage_get(float error)
{
    float a;

    a = error < 0.0f ? -error : error;
    if (a <= 30.0f) /* 直道：误差小，保持基础速度 */
    {
        return 0;
    }
    if (a >=200.0f ) /* 大误差：进入直角弯阶段 */
    {
        return 2;
    }
    return 1;
}

void steer_task(void)
{
    float target;
    SteerStage_t *stage;
    uint8 stage_id;
    uint32 now;
    bit irq_state;

    if (!steer_running) return;
    /* 32 位计数由中断更新，读取时保留原中断状态。 */
    irq_state = EA;
    EA = 0;
    now = scheduler_tick;
    EA = irq_state;
    steer_in = track_error();
    if (turn.active)
    {
        if (speed_enable && (uint32)(now - turn.start) < steer_plan.turn_ms)
        {
            steer_out = turn.target;
            yaw_angle_set(steer_out, steer_plan.turn.speed, steer_plan.turn.out_max);
            return;
        }
        turn.active = 0;
        /* 恢复实时循迹时对齐历史误差，避免保持期间的误差跳变产生微分冲击。 */
        pid_steer.integral = 0.0f;
        pid_steer.error = steer_in;
        pid_steer.last_error = steer_in;
    }
    if (!track.valid)
    {
        steer_stage_id = 0;
        steer_in = 0.0f;
        steer_out = 0.0f;
        if (speed_enable)
        {
            /* 无有效灰度时按直道速度继续，目标角速度归零。 */
            yaw_angle_set(0.0f, steer_plan.straight.speed, yaw_param.out_max);
        }
        return;
    }
    stage_id = steer_stage_get(steer_in);
    if (stage_id == 0U) turn.ready = 1;
    if (stage_id == 0)
    {
        /* 结构体字段明确对应参考工程的 normal/straight 策略。 */
        stage = &steer_plan.straight;
    }
    else if (stage_id == 1)
    {
        stage = &steer_plan.curve;
    }
    else
    {
        stage = &steer_plan.turn;
    }
    if (stage_id != steer_stage_id)
    {
        steer_stage_id = stage_id;
        /* 不让上一阶段的积分和历史误差带入新阶段。 */
        pid_steer.integral = 0.0f;
        pid_steer.last_error = pid_steer.error;
    }
    pid_steer.kp = stage->kp;
    pid_steer.ki = stage->ki;
    pid_steer.kd = stage->kd;
    pid_steer.limit = stage->out_max;
    target = STEER_YAW_RATE_SIGN * pid_calculate_positional(&pid_steer,
                                                             -steer_in);
    /* 寻迹环和角速度环统一使用 deg/s。 */
    steer_out = pid_constrain(target, -stage->out_max, stage->out_max);
    if (speed_enable)
    {
        if (stage_id == 2U && turn.ready && steer_plan.turn_ms > 0U)
        {
            turn.start = now;
            turn.target = steer_out;
            turn.active = 1;
            turn.ready = 0;
        }
        yaw_angle_set(steer_out, stage->speed, stage->out_max);
    }
}

void ctrl_init(void)
{
    motor_init();
    encoder_init();
    track_init();
    imu_init();
    pid_init_all();
    yaw_angle_init();
    steer_init();
    speed_enable = 0;
}

void sensor_task(void)
{
    encoder_sample();
    imu_task();
}

void control_tick_isr(void)
{
    /* 先更新反馈，再运行角速度环和速度环。 */
    sensor_task();
    yaw_task();
    speed_task();
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
