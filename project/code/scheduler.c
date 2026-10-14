#include "scheduler.h"
#include "board_cfg.h"
#include "control.h"
#include "key.h"
#include "screen.h"

volatile uint32 scheduler_tick = 0;
uint8 task_num = 0;

typedef struct
{
    void (*run)(void);
    uint16 rate_ms;
    uint32 last_ms;
} task_t;

void key_proc(void);

task_t tasks[] =
{
    {track_task,    TASK_MS,     0},
    {key_proc,      KEY_SCAN_MS, 0},
    {pid_wave_task, PID_WAVE_MS, 0},
    {screen_task,   SCREEN_MS,   0}
};

void scheduler_tick_isr(void)
{
    /* TIM4 只维护 1 ms 系统时基。 */
    scheduler_tick++;
}

void control_tick_isr(void)
{
    /* 5 ms 内环：传感器 -> 角速度环 -> 速度环；寻迹目标由主循环更新。 */
    sensor_task();
    yaw_task();
    speed_task();
}

void key_proc(void)
{
    key_task();
    if (key_down & KEY_B3)
    {
        key_down &= (uint8)~KEY_B3;
        if (speed_enable)
        {
            speed_stop();
        }
        else
        {
            speed_start();
        }
    }
}

void scheduler_init(void)
{
    uint8 i;

    scheduler_tick = 0;
    task_num = (uint8)(sizeof(tasks) / sizeof(tasks[0]));
    for (i = 0; i < task_num; i++)
    {
        tasks[i].last_ms = 0;
    }

    key_init();

    pit_ms_init(SCHEDULER_PIT, 1, scheduler_tick_isr);
    pit_ms_init(CONTROL_PIT, TASK_MS, control_tick_isr);
}

void scheduler_run(void)
{
    uint8 i;
    uint32 now;

    interrupt_global_disable();
    now = scheduler_tick;
    interrupt_global_enable();

    for (i = 0; i < task_num; i++)
    {
        if ((uint32)(now - tasks[i].last_ms) >= tasks[i].rate_ms)
        {
            tasks[i].last_ms += tasks[i].rate_ms;
            tasks[i].run();
        }
    }
}
