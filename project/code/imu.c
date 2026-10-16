#include "imu.h"
#include "board_cfg.h"

float yaw_rate = 0.0f;
uint8 imu_ok = 0;

void imu_init(void)
{
    imu_ok = 1;
    if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE))
    {
        imu_ok = 0;
    }
}

void imu_task(void)
{
    imu660rc_get_gyro();
    /* IMU驱动和角速度环统一使用 deg/s。 */
    yaw_rate = IMU_YAW_RATE_SIGN * imu660rc_gyro_transition(imu660rc_gyro_z);
}
