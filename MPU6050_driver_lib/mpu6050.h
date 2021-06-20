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
#include "inv_mpu.h"
/* Private macros ------------------------------------------------------------*/
//gyro fsr
#define GYRO_FSR_250DPS 	250
#define GYRO_FSR_500DPS 	500
#define GYRO_FSR_1000DPS 	1000
#define GYRO_FSR_2000DPS 	2000
//accel fsr
#define  ACCEL_FSR_2G		2
#define  ACCEL_FSR_4G		4
#define  ACCEL_FSR_8G		8
#define  ACCEL_FSR_16G		16
//lpf
#define LPF_5HZ			5
#define LPF_10HZ		10
#define LPF_20HZ		20
#define LPF_42HZ		42
#define LPF_98HZ		98
#define LPF_188HZ		188
/* Private type --------------------------------------------------------------*/
struct mpu_config_s {
	unsigned char sensors;   		//Select which sensors are pushed to FIFO.
	unsigned short fifo_config;		//Select which sensors are pushed to FIFO.
	unsigned short sample_rate;
	unsigned char accel_fsr;		//Set the accel full-scale range.
	unsigned short gyro_fsr;		//Set the gyro full-scale range.
	unsigned char lpf;				//Set digital low pass filter
	signed char gyro_orientation[9];
};

struct dmp_config_s {
	unsigned char dmp_on;
	unsigned short dmp_feature;
	unsigned char interrupt_mode;
	unsigned short fifo_rate;
};

struct mpu_rec_s {
	float gyro[3];//x,y,z
	float accel[3];//x,y,z
	long  quat[4];
	float pitch;
	float roll;
	float yaw;
	short sensors;
	unsigned char more;
	unsigned long sensor_timestamp;
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern struct mpu_config_s mpu_config;
extern struct dmp_config_s dmp_config;
/* Exported function declarations --------------------------------------------*/
uint8_t MPU6050_Config_Pin(GPIO_TypeDef *gpiox, uint16_t scl_pinx, uint16_t sda_pinx);
unsigned char MPU6050_Init(struct mpu_config_s *_mpu_s,struct dmp_config_s *_dmp_s);
void MPU6050_run_self_test(uint8_t set_accel_bias);
uint8_t  dmp_read_data(struct mpu_rec_s *_rec_s);
uint8_t mpu_read_data(struct mpu_rec_s* _rec_s);


#ifdef  __cplusplus
}
#endif
#endif /*__MPU_6050_H */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
