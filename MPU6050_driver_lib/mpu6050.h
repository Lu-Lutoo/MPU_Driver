  /**
    ******************************************************************************
    * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
    * @file    mpu6050.h
    * @author  Lutoo e19135693@163.com
    * @brief   Header file of mpu6050.
    ******************************************************************************
    * @attention
    * 
    * if you had modified this file, please make sure your code does not have many 
    * bugs, update the version Number, write dowm your name and the date, the most
    * important is make sure the users will have clear and definite understanding 
    * through your new brief.
    *
    * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
    * All rights reserved.</center></h2>
    ******************************************************************************
    */
#ifndef __MPU_6050_H
#define __MPU_6050_H

#ifdef  __cplusplus
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/
#include "mpu_component.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
struct mpu_s {
	unsigned char sensors;
	unsigned short fifo_config;
	unsigned short sample_rate; //must be between 4Hz to 1kHz.我应该用一个断言检测(不用了，官方自己限位了)
	unsigned char accel_fsr;
	unsigned short gyro_fsr;
	unsigned char lpf;
	signed char gyro_orientation[9];
};

struct dmp_s {
	unsigned char dmp_on;
	unsigned short dmp_feature;
	unsigned char interrupt_mode;
	unsigned short fifo_rate;
	unsigned char gyro_cal_on; //1 for on, 0 for off
};

struct rec_s {
	short gyro[3];
	short accel[3];
	long  quat[4];
	float angle[3];
	short sensors;
	unsigned char more;
	unsigned long sensor_timestamp;
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern struct mpu_s mpu_config;
extern struct dmp_s dmp_config;
extern struct rec_s mpu_receive;
/* Exported function declarations --------------------------------------------*/
uint8_t MPU6050_Config_Pin(GPIO_TypeDef *gpiox, uint16_t scl_pinx, uint16_t sda_pinx);
unsigned char MPU6050_Init(struct mpu_s *_mpu_s,struct dmp_s *_dmp_s,struct rec_s *_rec_s);
static inline void run_self_test(void);
void dmp_gyro_cal(struct dmp_s *_dmp_s);
void  dmp_read_data(struct rec_s *_rec_s);



#ifdef  __cplusplus
}
#endif
#endif /*__MPU_6050_H */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
