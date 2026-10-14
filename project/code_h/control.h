#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "zf_common_headfile.h"

/* 通用 PID 状态。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float out;
    float limit;
    float error;
    float last_error;
    float last_feedback;
    float differential;
    float integral;
    float p_out;
    float i_out;
    float d_out;
} PID_T;

/* 速度环参数。 */
typedef struct
{
    float kf;
    float kp;
    float ki;
    float kd;
    float i_max;
    float err_max;
    float d_hz;
    float dt;
    float out_min;
    float out_max;
} SpeedPid_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float out_max;
    float filter_alpha;
} YawParam_t;

typedef struct
{
    float target_yaw_rate;
    float base_speed;
    float out_max;
    float diff_speed;
    uint8 running;
} YawCtrl_t;

extern PID_T pid_speed_left;
extern PID_T pid_speed_right;
extern PID_T pid_steer;
extern PID_T pid_yaw;
extern SpeedPid_t left;
extern SpeedPid_t right;
extern YawParam_t yaw_param;
extern YawCtrl_t yaw_ctrl;
extern float yaw_rate;
extern uint8 imu_ok;
extern uint8 pid_running;
extern uint8 steer_running;
extern volatile uint8 speed_enable;

void ctrl_init(void);
void pid_init_all(void);
void speed_set(float l, float r);
void speed_start(void);
void speed_stop(void);
void speed_pid_task(void);
void pid_wave_task(void);
void steer_init(void);
void steer_task(void);
void yaw_angle_init(void);
void yaw_angle_set(float target_yaw_rate, float base_speed, float out_max);
void yaw_angle_task(void);

void sensor_task(void);
void track_task(void);
void yaw_task(void);
void speed_task(void);

float pid_constrain(float value, float low, float high);
void pid_init(PID_T *pid, float kp, float ki, float kd, float target, float limit);
void pid_set_target(PID_T *pid, float target);
void pid_set_limit(PID_T *pid, float limit);
float pid_calculate_positional(PID_T *pid, float current);

#endif
