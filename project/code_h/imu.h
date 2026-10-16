#ifndef APP_IMU_H
#define APP_IMU_H

#include "zf_common_headfile.h"

extern float yaw_rate;
extern uint8 imu_ok;

void imu_init(void);
void imu_task(void);

#endif
