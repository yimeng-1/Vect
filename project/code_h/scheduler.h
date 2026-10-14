#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "zf_common_headfile.h"

void scheduler_init(void);
void scheduler_run(void);
void scheduler_tick_isr(void);
void control_tick_isr(void);

#endif
