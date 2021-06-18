  /**
    ******************************************************************************
    * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
    * @file    Templete.h
    * @author  Lutoo e19135693@163.com
    * @brief   Header file of Templete.
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
#ifndef __MPU_COMPONENT_H
#define __MPU_COMPONENT_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "Drivers/Components/drv_i2c.h"
#include "Drivers/Components/drv_timer.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations --------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern IIC_PIN_Typedef MPU_IIC_PIN;
/* Exported function declarations --------------------------------------------*/
unsigned char MPU_I2C_WriteRegister(unsigned char slave_addr,unsigned char reg_addr,
                              unsigned short len, unsigned char *data_ptr);
unsigned char MPU_I2C_ReadRegister(unsigned char Address,unsigned char RegisterAddr, 
                             unsigned short RegisterLen,unsigned char *RegisterValue);
int get_tick_count(unsigned long *count);
	


#ifdef __cplusplus
}
#endif
#endif /* __MPU_COMPONENT_H */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
