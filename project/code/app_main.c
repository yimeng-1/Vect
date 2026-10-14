#include "zf_common_headfile.h"
#include "scheduler.h"
#include "control.h"

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    ctrl_init();
    scheduler_init();

    while (1)
    {
        scheduler_run();
    }
}
