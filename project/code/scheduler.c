#include "scheduler.h"
#include "board_cfg.h"
#include "key.h"
#include "screen.h"
#include "track.h"

volatile uint32 scheduler_tick = 0;
uint8 task_num = 0;

typedef struct
{
    void (*run)(void);
    uint16 rate_ms;
    uint32 last_ms;
} task_t;

task_t tasks[] =
{
    {track_task,    TASK_MS,     0},
    {key_proc,      KEY_SCAN_MS, 0},
    {screen_task,   SCREEN_MS,   0}
};

void scheduler_tick_isr(void)
{
    /* TIM4 只维护 1 ms 系统时基。 */
    scheduler_tick++;
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
