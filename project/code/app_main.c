#include "zf_common_headfile.h"
#include "scheduler.h"
#include "control.h"
#include "key.h"
#include "board_cfg.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    ctrl_init();
    scheduler_init();
    key_init();
    pit_ms_init(SCHEDULER_PIT, 1, scheduler_tick_isr);
    pit_ms_init(CONTROL_PIT, TASK_MS, control_tick_isr);

    while (1)
    {
        scheduler_run();
    }
}
