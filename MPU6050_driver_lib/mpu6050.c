/**
    ******************************************************************************
    * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
    * @file    mpu6050.c
    * @author  Lutoo e19135693@163.com
    * @brief   Code for mpu6050.
    * @date    2021-04-25
    * @version 1.0
    * @par Change Log：
    * <table>
    * <tr><th>Date        <th>Version  <th>Author    		<th>Description
    * <tr><td>2021-04-25  <td> 1.0     <td>Lutoo      		<td>Creator
    * </table>
    *
    ==============================================================================
                        ##### How to use this driver #####
    ==============================================================================
      @note
	 	 -# To initialize the DMP:
		 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
		 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
		 * 2. Push the gyro and accel orientation matrix to the DMP.
		 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
		 *    executed unless the corresponding feature is enabled.
		 * 4. Call dmp_enable_feature(mask) to enable different features.
		 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
		 * 6. Call any feature-specific control functions.
		 *
		 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
		 * be called repeatedly to enable and disable the DMP at runtime.
		 *
		 * The following is a short summary of the features supported in the DMP
		 * image provided in inv_mpu_dmp_motion_driver.c:
		 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
		 * 200Hz. Integrating the gyro data at higher rates reduces numerical
		 * errors (compared to integration on the MCU at a lower sampling rate).
		 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
		 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
		 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
		 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
		 * an event at the four orientations where the screen should rotate.
		 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
		 * no motion.
		 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
		 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
		 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
		 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.

      @warning
        -#* Known Bug -
		 * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
		 * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
		 * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
		 * there will be a 25Hz interrupt from the MPU device.
		 *
		 * There is a known issue in which if you do not enable DMP_FEATURE_TAP
		 * then the interrupts will be at 200Hz even if fifo rate
		 * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
				
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
/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"

#include <math.h>
#include <string.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* Private define ------------------------------------------------------------*/
//dmp FIFO default  rate
const unsigned short DMP_FIFO_DEFAULT_RATE = 200;
//gyro fsr
const unsigned short GYRO_FSR_250DPS 	= 250;
const unsigned short GYRO_FSR_500DPS 	= 500;
const unsigned short GYRO_FSR_1000DPS 	= 1000;
const unsigned short GYRO_FSR_2000DPS 	= 2000;
//accel fsr
const unsigned char  ACCEL_FSR_2G		= 2;
const unsigned char  ACCEL_FSR_4G		= 4;
const unsigned char  ACCEL_FSR_8G		= 8;
const unsigned char  ACCEL_FSR_16G		= 16;
//lpf
const unsigned short LPF_5HZ			= 5;
const unsigned short LPF_10HZ			= 10;
const unsigned short LPF_20HZ			= 20;
const unsigned short LPF_42HZ			= 42;
const unsigned short LPF_98HZ			= 98;
const unsigned short LPF_188HZ			= 188;

/* Private variables ---------------------------------------------------------*/
const signed char default_gyro_orientation[9] = {-1, 0, 0,
                                           	      0,-1, 0,
											      0, 0, 1};
/* Private type --------------------------------------------------------------*/
//mpu config example , I will delete it if I complete the library
struct mpu_s mpu_config = {
		.sensors 		= INV_XYZ_GYRO|INV_XYZ_ACCEL,
		.fifo_config 	= INV_XYZ_GYRO|INV_XYZ_ACCEL,
		.sample_rate 	= DMP_FIFO_DEFAULT_RATE,  //200Hz
		.accel_fsr		= ACCEL_FSR_8G,
		.gyro_fsr		= GYRO_FSR_1000DPS,
		.lpf 			= 0,   // close low pass filter
		.gyro_orientation = {-1, 0, 0,
                			  0,-1, 0,
							  0, 0, 1}
};

//dmp config example , I will delete it if I complete the library
struct dmp_s dmp_config = {
		.dmp_on = 1,
		.dmp_feature = DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_GYRO_CAL,
		.interrupt_mode = DMP_INT_CONTINUOUS,
		.fifo_rate = DMP_FIFO_DEFAULT_RATE, //dmp sample rate 是初始化时固定为200Hz的，要设置输出fifo输出频率必须比它低，但官方没说最低多少
		.gyro_cal_on = 1
};

// mpu receive data packet
struct rec_s mpu_receive = {0};


/* Private function declarations ---------------------------------------------*/
static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
uint16_t get_gpio_pin_num(uint16_t GPIO_Pin);
/* function prototypes -------------------------------------------------------*/

/**
  * @brief  
  * @param  
  * @retval 
	*/

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

uint16_t get_gpio_pin_num(uint16_t GPIO_Pin)
{
  uint16_t x = 0;
  while (GPIO_Pin >>= 1)x++;
  return x;
}

uint8_t MPU6050_Config_Pin(GPIO_TypeDef *gpiox, uint16_t scl_pinx, uint16_t sda_pinx)
{
	if(!IS_GPIO_PIN(scl_pinx) || (scl_pinx & (scl_pinx - 1)))return 1;
	if(!IS_GPIO_PIN(sda_pinx) || (sda_pinx & (sda_pinx - 1)))return 1;

	/* MPU6050_IIC initialization */
	MPU_IIC_PIN.IIC_GPIO_PORT = gpiox;
	MPU_IIC_PIN.IIC_SCL_PIN = scl_pinx;
	MPU_IIC_PIN.IIC_SDA_PIN = sda_pinx;

	MPU_IIC_PIN.IIC_SCL_PIN_NUM = get_gpio_pin_num(scl_pinx);
	MPU_IIC_PIN.IIC_SDA_PIN_NUM = get_gpio_pin_num(sda_pinx);

	return 0;
}



unsigned char MPU6050_Init(struct mpu_s *_mpu_s,struct dmp_s *_dmp_s,struct rec_s *_rec_s)
{
	/* initial iic bus */
	IIC_Init(&MPU_IIC_PIN);

	/* mpu init */
	if(mpu_init()){
		delay_ms_nos(100);
		__set_FAULTMASK(1); //reset
		NVIC_SystemReset();}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake sensors. */
	mpu_set_sensors(_mpu_s->sensors);
	/* config mpu FIFO. */
	mpu_configure_fifo(_mpu_s->fifo_config);
	/* config mou sample rate */
	mpu_set_sample_rate(_mpu_s->sample_rate);
	/* set accel fsr and gyro fsr */
	mpu_set_accel_fsr(_mpu_s->accel_fsr);
	mpu_set_gyro_fsr(_mpu_s->gyro_fsr);
	/* set low pass filter */
	if(_mpu_s->lpf){
		mpu_set_lpf(_mpu_s->lpf);}

	if(!_dmp_s->dmp_on){return 0;}
	/* initialize the DMP */
	/* Load the DMP firware */
	dmp_load_motion_driver_firmware();
	/* Push gyro and accel orientation to the DMP */
	dmp_set_orientation(
	        inv_orientation_matrix_to_scalar(_mpu_s->gyro_orientation));
	/* Enable DMP features */
	dmp_enable_feature(_dmp_s->dmp_feature);
	/* config dmp fifo rate */
	if(_dmp_s->fifo_rate && _dmp_s->fifo_rate < 200){
		dmp_set_fifo_rate(_dmp_s->fifo_rate);}
	/* config dmp interrupt mode */
	if(_dmp_s->interrupt_mode){
		dmp_set_interrupt_mode(_dmp_s->interrupt_mode);}
	/* turn on dmp */
	mpu_set_dmp_state(1);

	/* Initialize receive variables. */
	memset(_rec_s, 0, sizeof(*_rec_s));

	return 0;
}

//初始化结束后再用
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    unsigned char i = 0;
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }
        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_6050_reg(accel);
    }
}

//when you use this function ,mpu should stay eight seconds of no motion
void dmp_gyro_cal(struct dmp_s *_dmp_s)
{
	dmp_enable_gyro_cal(_dmp_s->gyro_cal_on);
}


void  dmp_read_data(struct rec_s *_rec_s)
{
	dmp_read_fifo(_rec_s->gyro, _rec_s->accel,_rec_s->quat,
			&(_rec_s->sensor_timestamp), &(_rec_s->sensors),&(_rec_s->more));
	long t1, t2, t3;
	long q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
	float values[3];

	q00 = (_rec_s->quat[0]*_rec_s->quat[0])>>29;
	q01 = (_rec_s->quat[0]*_rec_s->quat[1])>>29;
	q02 = (_rec_s->quat[0]*_rec_s->quat[2])>>29;
	q03 = (_rec_s->quat[0]*_rec_s->quat[3])>>29;
	q11 = (_rec_s->quat[1]*_rec_s->quat[1])>>29;
	q12 = (_rec_s->quat[1]*_rec_s->quat[2])>>29;
	q13 = (_rec_s->quat[1]*_rec_s->quat[3])>>29;
	q22 = (_rec_s->quat[2]*_rec_s->quat[2])>>29;
	q23 = (_rec_s->quat[2]*_rec_s->quat[3])>>29;
	q33 = (_rec_s->quat[3]*_rec_s->quat[3])>>29;

	/* X component of the Ybody axis in World frame */
	t1 = q12 - q03;

	/* Y component of the Ybody axis in World frame */
	t2 = q22 + q00 - (1L << 30);
	values[2] = -atan2f((float) t1, (float) t2) * 180.f / (float) M_PI;

	/* Z component of the Ybody axis in World frame */
	t3 = q23 + q01;
	values[0] = atan2f((float) t3,sqrtf((float) t1 * t1 +
					  (float) t2 * t2)) * 180.f / (float) M_PI;
	/* Z component of the Zbody axis in World frame */
	t2 = q33 + q00 - (1L << 30);
	if (t2 < 0) {
		if (values[0] >= 0)
			values[0] = 180.f - values[0];
		else
			values[0] = -180.f - values[0];
	}

	/* X component of the Xbody axis in World frame */
	t1 = q11 + q00 - (1L << 30);
	/* Y component of the Xbody axis in World frame */
	t2 = q12 + q03;
	/* Z component of the Xbody axis in World frame */
	t3 = q13 - q02;

	values[1] =
		(atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) *
		  180.f / (float) M_PI - 90);
	if (values[1] >= 90)
		values[1] = 180 - values[1];

	if (values[1] < -90)
		values[1] = -180 - values[1];
	_rec_s->angle[0] = (long)(values[0] * 65536.f);
	_rec_s->angle[1] = (long)(values[1] * 65536.f);
	_rec_s->angle[2] = (long)(values[2] * 65536.f);
}




/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

