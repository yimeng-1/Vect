/*********************************************************************************************************************
* STC32G144K Opensource Library ����STC32G144K ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2025 SEEKFREE ��ɿƼ�
*
* ���ļ���STC32G144K��Դ���һ����
*
* STC32G144K ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MDK FOR C251
* ����ƽ̨          STC32G144K
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����           ��ע
* 2024-08-01        ��W            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   // Ӳ�� SPI ����
*                   SCL/SPC           �鿴 zf_device_imu660rc.h �� IMU660RC_SPC_PIN �궨��
*                   SDA/SDI           �鿴 zf_device_imu660rc.h �� IMU660RC_SDI_PIN �궨��
*                   SA0/SDO           �鿴 zf_device_imu660rc.h �� IMU660RC_SDO_PIN �궨��
*                   CS                �鿴 zf_device_imu660rc.h �� IMU660RC_CS_PIN �궨��
*										INT2              �鿴 zf_device_imu660rc.h �� IMU660RC_INT2_PIN  �궨��
*                   VCC               3.3V��Դ
*                   GND               ��Դ��
*                   ������������
*
*                   // ���� IIC ����
*                   SCL/SPC           �鿴 zf_device_imu660rc.h �� IMU660RC_SCL_PIN �궨��
*                   SDA/SDI           �鿴 zf_device_imu660rc.h �� IMU660RC_SDA_PIN �궨��
*                   VCC               3.3V��Դ
*                   GND               ��Դ��
*                   ������������
*                   ------------------------------------
********************************************************************************************************************/


#ifndef _zf_device_imu660rc_h_
#define _zf_device_imu660rc_h_

#include "zf_common_typedef.h"
#include "zf_device_type.h"



#define IMU660RC_USE_INTERFACE       HARDWARE_SPI                        // Ĭ��ʹ��Ӳ�� SPI ��ʽ����

#if (IMU660RC_USE_INTERFACE==HARDWARE_SPI)
//====================================================Ӳ�� SPI ����====================================================
	#define IMU660RC_SPI_SPEED          ((uint32)10 * 1000 * 1000U)  		// Ӳ�� SPI ����
	#define IMU660RC_SPI                ( SPI_3             )               // Ӳ�� SPI ��
	#define IMU660RC_SPC_PIN            ( SPI3_CH3_SCLK_P87 )             	// Ӳ�� SPI SCK ����
	#define IMU660RC_SDI_PIN            ( SPI3_CH3_MOSI_P85 )             	// Ӳ�� SPI MOSI ����
	#define IMU660RC_SDO_PIN            ( SPI3_CH3_MISO_P86 )              	// Ӳ�� SPI MISO ����
	#define IMU660RC_CS_PIN             ( IO_P34  )                       	// CS Ƭѡ����
	#define IMU660RC_CS(x)              ( P34 = x )
//====================================================Ӳ�� SPI ����====================================================
#elif (IMU660RC_USE_INTERFACE==SOFT_SPI)
//====================================================���� SPI ����====================================================
	#define IMU660RC_SPC_PIN            (P40)                           // ���� SPI SCK  ����
	#define IMU660RC_SDI_PIN            (P41)                           // ���� SPI MOSI ����
	#define IMU660RC_SDO_PIN            (P42)                           // ���� SPI MISO ����
	#define IMU660RC_CS_PIN             (P43)                        	// ���� SPI CS   ����
//====================================================���� SPI ����====================================================
#elif (IMU660RC_USE_INTERFACE==SOFT_IIC)
//====================================================���� IIC ����====================================================
	#define IMU660RC_SOFT_IIC_DELAY     (0)                             // ���� IIC ��ʱ����ʱ���� ��ֵԽС IIC ͨ������Խ��
	#define IMU660RC_SCL_PIN            (IO_P40)                        // ���� IIC SCL ���� ���� IMU660RC �� SCL ����
	#define IMU660RC_SDA_PIN            (IO_P41)                        // ���� IIC SDA ���� ���� IMU660RC �� SDA ����
//====================================================���� IIC ����====================================================
#endif

#define IMU660RC_INT2_PIN		( GPIO_INIT_P36 )	// �ж��ź����ţ��ڶ�ȡ��Ԫ��ʱ��Ҫʹ�ã��жϷ�ʽģʽ��Ҫ������


#define IMU660RC_QUARTERNION_GET_GYRO   ( 1 )                                   // 1���������Ԫ����ģʽʱ����ȡ��Ԫ��ʱ�Զ���ȡ���ٶ� 0�����Զ���ȡ
#define IMU660RC_QUARTERNION_GET_ACC    ( 1 )                                   // 1���������Ԫ����ģʽʱ����ȡ��Ԫ��ʱ�Զ���ȡ���ٶ� 0�����Զ���ȡ
#define IMU660RC_ACC_SAMPLE_DEFAULT     ( IMU660RC_ACC_SAMPLE_SGN_8G )          // ��������Ĭ�ϵ� ���ٶȼ� ��ʼ������
#define IMU660RC_GYRO_SAMPLE_DEFAULT    ( IMU660RC_GYRO_SAMPLE_SGN_2000DPS )    // ��������Ĭ�ϵ� ������   ��ʼ������

typedef enum
{
    IMU660RC_MAIN_MEM_BANK  = 0x00,
    IMU660RC_HUB_MEM_BANK   = 0x40,
    IMU660RC_EMBED_MEM_BANK = 0x80,
}imu660rc_mem_bank_enum;

typedef enum
{
    IMU660RC_ACC_SAMPLE_SGN_2G ,                                                // ���ٶȼ����� ��2G  (ACC = Accelerometer ���ٶȼ�) (SGN = signum �������� ��ʾ������Χ) (G = g �������ٶ� g��9.80 m/s^2)
    IMU660RC_ACC_SAMPLE_SGN_4G ,                                                // ���ٶȼ����� ��4G  
    IMU660RC_ACC_SAMPLE_SGN_8G ,                                                // ���ٶȼ����� ��8G  
    IMU660RC_ACC_SAMPLE_SGN_16G,                                                // ���ٶȼ����� ��16G 
}imu660rc_acc_sample_config;

typedef enum
{
    IMU660RC_GYRO_SAMPLE_SGN_125DPS ,                                           // ���������� ��125DPS  (GYRO = Gyroscope ������) (SGN = signum �������� ��ʾ������Χ) (DPS = Degree Per Second ���ٶȵ�λ ��/S)
    IMU660RC_GYRO_SAMPLE_SGN_250DPS ,                                           // ���������� ��250DPS  
    IMU660RC_GYRO_SAMPLE_SGN_500DPS ,                                           // ���������� ��500DPS  
    IMU660RC_GYRO_SAMPLE_SGN_1000DPS,                                           // ���������� ��1000DPS 
    IMU660RC_GYRO_SAMPLE_SGN_2000DPS,                                           // ���������� ��2000DPS 
    IMU660RC_GYRO_SAMPLE_SGN_4000DPS,                                           // ���������� ��4000DPS 
}imu660rc_gyro_sample_config;

typedef enum
{
    IMU660RC_QUARTERNION_15HZ,                                                  // 15 Hz
    IMU660RC_QUARTERNION_30HZ,                                                  // 30 Hz
    IMU660RC_QUARTERNION_60HZ,                                                  // 60 Hz
    IMU660RC_QUARTERNION_120HZ,                                                 // 120Hz
    IMU660RC_QUARTERNION_240HZ,                                                 // 240Hz
    IMU660RC_QUARTERNION_480HZ,                                                 // 480Hz
    IMU660RC_QUARTERNION_DISABLE,                                               // ������Ԫ�����
}imu660rc_quarternion_rate_config;


//================================================���� IMU660RC �ڲ���ַ================================================
#define IMU660RC_DEV_ADDR           ( 0x6B )                                    // SA0�ӵأ�0x6A SA0������0x6B ģ��Ĭ������
#define IMU660RC_SPI_W              ( 0x00 )
#define IMU660RC_SPI_R              ( 0x80 )
#define IMU660RC_TIMEOUT_COUNT      ( 0x00FF )                                  // IMU660RC ��ʱ����


//================================================���� IMU660RC �Ĵ�����ַ================================================
#define IMU660RC_FUNC_CFG_ACCESS    ( 0x01 )
#define IMU660RC_FIFO_CRTL1         ( 0x07 )
#define IMU660RC_FIFO_CRTL2         ( 0x08 )
#define IMU660RC_FIFO_CRTL3         ( 0x09 )
#define IMU660RC_FIFO_CRTL4         ( 0x0A )
#define IMU660RC_INT2_CTRL          ( 0x0E )
#define IMU660RC_CHIP_ID            ( 0x0F )
#define IMU660RC_CTRL1              ( 0x10 )
#define IMU660RC_CTRL2              ( 0x11 )
#define IMU660RC_CTRL3              ( 0x12 )
#define IMU660RC_CTRL4              ( 0x13 )
#define IMU660RC_CTRL5              ( 0x14 )
#define IMU660RC_CTRL6              ( 0x15 )
#define IMU660RC_CTRL7              ( 0x16 )
#define IMU660RC_CTRL8              ( 0x17 )
#define IMU660RC_CTRL9              ( 0x18 )
#define IMU660RC_CTRL10             ( 0x19 )
#define IMU660RC_CTRL_STATUS        ( 0x1A )
#define IMU660RC_STATUS_REG         ( 0x1E )
#define IMU660RC_OUT_TEMP_L         ( 0x20 )
#define IMU660RC_OUT_TEMP_H         ( 0x21 )
#define IMU660RC_OUTX_L_G           ( 0x22 )
#define IMU660RC_OUTX_H_G           ( 0x23 )
#define IMU660RC_OUTY_L_G           ( 0x24 )
#define IMU660RC_OUTY_H_G           ( 0x25 )
#define IMU660RC_OUTZ_L_G           ( 0x26 )
#define IMU660RC_OUTZ_H_G           ( 0x27 )
#define IMU660RC_OUTX_L_A           ( 0x28 )
#define IMU660RC_OUTX_H_A           ( 0x29 )
#define IMU660RC_OUTY_L_A           ( 0x2A )
#define IMU660RC_OUTY_H_A           ( 0x2B )
#define IMU660RC_OUTZ_L_A           ( 0x2C )
#define IMU660RC_OUTZ_H_A           ( 0x2D )

#define IMU660RC_PAGE_SEL           ( 0x02 )
#define IMU660RC_EMB_FUNC_EN_A      ( 0x04 )
#define IMU660RC_PAGE_RW            ( 0x17 )      
#define IMU660RC_EMB_FUNC_FIFO_EN_A ( 0x44 )  
#define IMU660RC_SFLP_ODR           ( 0x5E )
#define IMU660RC_EMB_FUNC_CFG       ( 0x63 )



extern float imu660rc_transition_factor[2];
extern int16 imu660rc_gyro_x,   imu660rc_gyro_y,    imu660rc_gyro_z;    // ��������������  
extern int16 imu660rc_acc_x ,   imu660rc_acc_y ,    imu660rc_acc_z;     // ������ٶȼ�����
extern float imu660rc_roll  ,   imu660rc_pitch ,    imu660rc_yaw;       // ŷ����
extern float imu660rc_quarternion[4];                                   // ��Ԫ��


void    imu660rc_get_acc            (void);
void    imu660rc_get_gyro           (void);
void    imu660rc_get_quarternion    (void);

//-------------------------------------------------------------------------------------------------------------------
// �������     �� IMU660RC ���ٶȼ�����ת��Ϊʵ����������
// ����˵��     acc_value       ������ļ��ٶȼ�����
// ���ز���     void
// ʹ��ʾ��     float data = imu660rc_acc_transition(imu660rc_acc_x);           // ��λΪ g(m/s^2)
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
#define imu660rc_acc_transition(acc_value)      ((float)(acc_value) / imu660rc_transition_factor[0])

//-------------------------------------------------------------------------------------------------------------------
// �������     �� IMU660RC ����������ת��Ϊʵ����������
// ����˵��     gyro_value      �����������������
// ���ز���     void
// ʹ��ʾ��     float data = imu660rc_gyro_transition(imu660rc_gyro_x);         // ��λΪ ��/s
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
#define imu660rc_gyro_transition(gyro_value)    ((float)(gyro_value) / imu660rc_transition_factor[1])
void    imu660rc_callback           (void);
uint8   imu660rc_init               (imu660rc_quarternion_rate_config quarternion_rate);



#endif
