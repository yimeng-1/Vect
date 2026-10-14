#ifndef BOARD_CFG_H
#define BOARD_CFG_H

#include "zf_common_headfile.h"

/* DRV8701E PH/EN 模式：EN 接 PWM，PH 控制方向。 */
#define MOTOR_PWM_FREQ       (17000UL)       /* PWM 频率 */
#define MOTOR_DUTY_MAX       (PWM_DUTY_MAX)  /* 最大占空比 */
#define DEAD_LEFT            (780)           /* 左电机实测死区，用于速度环补偿 */
#define DEAD_RIGHT           (850)           /* 右电机实测死区，用于速度环补偿 */
#define MOTOR_LEFT_DIR       (IO_P51)        /* 左电机 PH：方向 */
#define MOTOR_LEFT_PWM       (PWMD_CH1_P50)  /* 左电机 EN：PWM */
#define MOTOR_RIGHT_DIR      (IO_P53)        /* 右电机 PH：方向 */
#define MOTOR_RIGHT_PWM      (PWMD_CH3_P52)  /* 右电机 EN：PWM */
#define MOTOR_LEFT_SIGN      (-1)            /* 左方向修正：电机正转与编码器正方向相反 */
#define MOTOR_RIGHT_SIGN     (-1)             /* 右方向修正：正输出对应右轮向前 */

/* 单路脉冲加方向编码器：软件通道按实际接线交换。 */
#define ENCODER_LEFT_INDEX   (TIM18_ENCODER)          /* 左变量读取实际左轮通道 */
#define ENCODER_LEFT_PULSE   (TIM18_ENCODER_CH1_P90)  /* 左变量脉冲输入 */
#define ENCODER_LEFT_DIR     (IO_P46)                 /* 左变量方向输入 */
#define ENCODER_RIGHT_INDEX  (TIM17_ENCODER)          /* 右变量读取实际右轮通道 */
#define ENCODER_RIGHT_PULSE  (TIM17_ENCODER_CH1_P80)  /* 右变量脉冲输入 */
#define ENCODER_RIGHT_DIR    (IO_P44)                 /* 右变量方向输入 */
#define ENCODER_LEFT_SIGN    (1)                      /* 左编码器方向修正 */
#define ENCODER_RIGHT_SIGN   (-1)                     /* 右轮向前时原始计数为负，取反为正反馈 */
#define ENCODER_LEFT_PULSE_PER_M  (23381.0f)          /* 左轮行驶 1 m 的脉冲数 */
#define ENCODER_RIGHT_PULSE_PER_M (22795.0f)          /* 右轮行驶 1 m 的脉冲数 */

/* 龙邱 16 路灰度：S0~S3 选通道，AS 输出被选中的模拟值。 */
#define TRACK_S3             (IO_P03)          /* OUT4，选择位 bit3 */
#define TRACK_S2             (IO_P02)          /* OUT3，选择位 bit2 */
#define TRACK_S1             (IO_P01)          /* OUT2，选择位 bit1 */
#define TRACK_S0             (IO_P00)          /* OUT1，选择位 bit0 */
#define TRACK_AS             (ADC2_CH4_P04)    /* AS，ADC 模拟输入 */
#define TRACK_INV            (0U)              /* 0：原始值为线；1：反相 */
#define TRACK_GAIN           (10.0f)           /* 循迹误差放大倍数，屏幕和寻迹环共用 */
#define TRACK_SUM_MIN        (500UL)           /* 16 路原始信号总和有效门限 */
#define TRACK_TEST_ENABLE    (0U)              /* 1：固定误差测试；0：使用灰度误差 */
#define TRACK_TEST_ERROR     (4.0f)            /* 固定送入寻迹环的测试误差 */

/* 主板按键：低电平按下。 */
#define KEY_B1_PIN           (IO_PB2)  /* 按键 B1 */
#define KEY_B2_PIN           (IO_PB3)  /* 按键 B2 */
#define KEY_B3_PIN           (IO_PB4)  /* 按键 B3 */
#define KEY_SCAN_MS          (10U)     /* 按键扫描周期 */

/* TIM4 提供调度时基，寻迹在主循环，角速度和速度环在 TIM3。 */
#define SCHEDULER_PIT        (TIM4_PIT)  /* 调度器 1 ms 时基 */
#define CONTROL_PIT          (TIM3_PIT)  /* 角速度和速度环控制定时器 */
#define TASK_MS              (5U)        /* 循迹外环和内环控制周期 */
#define PID_WAVE_MS          (20U)       /* PID 波形发送周期 */
#define SCREEN_MS            (100U)      /* 屏幕状态刷新周期 */
#define ENCODER_LEFT_SPEED_SCALE  (100000.0f / (ENCODER_LEFT_PULSE_PER_M * TASK_MS))  /* 左轮脉冲转 cm/s */
#define ENCODER_RIGHT_SPEED_SCALE (100000.0f / (ENCODER_RIGHT_PULSE_PER_M * TASK_MS)) /* 右轮脉冲转 cm/s */
#define SPEED_TEST_TARGET    (-5.0f)       /* B3 启动后的基础速度，单位 cm/s */
#define BASE_SPEED           (SPEED_TEST_TARGET) /* 循迹时的基础速度 */
#define MAX_SPEED            (300.0f)

/* IMU Z 轴和循迹转向方向修正。 */
#define IMU_YAW_RATE_SIGN    (1.0f)
#define STEER_YAW_RATE_SIGN  (-1.0f)        /* 线在右侧误差为负，转为正角速度右转 */
#define RAD_PER_DEG          (0.0174532925199433f)

#endif
