#include "zf_common_headfile.h"

/* 仅保留本项目实际使用的 PIT 和 IMU INT2 中断。 */
void INT2_IRQHandler(void) interrupt INT2_VECTOR
{
    if (int_irq_handlers[2] != NULL)
    {
        int_irq_handlers[2]();
    }
}

void TM4_IRQHandler(void) interrupt TMR4_VECTOR
{
    TIM4_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_4] != NULL)
    {
        tim_irq_handlers[TIM_4]();
    }
}
