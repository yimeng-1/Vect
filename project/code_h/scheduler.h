#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "zf_common_headfile.h"

extern volatile uint32 scheduler_tick; /* TIM4 的毫秒计数 */

void scheduler_init(void);
void scheduler_run(void);
void scheduler_tick_isr(void);

#endif
