# mpu6050_driver_lib

## How to use it?

### Three struct

#### mpu_s		

  + `sensors`  	 Select which sensors are pushed to FIFO.

    > INV_X_GYRO \ INV_Y_GYRO \ INV_Z_GYRO \ 
    > INV_XYZ_GYRO \ INV_XYZ_ACCEL \ INV_XYZ_COMPASS
    
  + `fifo_config`  	Select which sensors are pushed to FIFO.

	> INV_X_GYRO \ INV_Y_GYRO \ INV_Z_GYRO \ 
	> INV_XYZ_GYRO \ INV_XYZ_ACCEL \ INV_XYZ_COMPASS
	
  + `sample_rate` 

	>4~1000 开启dmp后采样率会被重新设置成与dmp采样率一致
	
  + `gyro_fsr`   Set the gyro full-scale range.

	> GYRO_FSR_250DPS \ GYRO_FSR_500DPS \ GYRO_FSR_1000DPS \ GYRO_FSR_2000DPS

  + `accel_fsr`	Set the accel full-scale range.

	> ACCEL_FSR_2G \ ACCEL_FSR_4G \ ACCEL_FSR_8G \ ACCEL_FSR_16G

  + `lpf` Set digital low pass filter

	> LPF_5HZ \ LPF_10HZ \ LPF_20HZ \ LPF_42HZ \ LPF_98HZ \ LPF_188HZ

  + `gyro_orientation` orientation matrix

	> 这个需要看mpu芯片放置和你需要的体坐标系的关系，你需要的体坐标系乘上orientation matrix会得到mpu芯片坐标系

#### dmp_s

+ `dmp_on` turn dmp on/off
+ `dmp_feature`

	> DMP_FEATURE_TAP
	> DMP_FEATURE_ANDROID_ORIENT
	> DMP_FEATURE_LP_QUAT
	> DMP_FEATURE_6X_LP_QUAT
	> DMP_FEATURE_GYRO_CAL
	> DMP_FEATURE_SEND_RAW_ACCEL
	> DMP_FEATURE_SEND_RAW_GYRO
	
	 *NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually exclusive.
	 NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also mutually exclusive.*

+ `interrupt_mode` 

	> DMP_INT_CONTINUOUS : One FIFO period has elapsed.
	> DMP_INT_GESTURE : A tap event has been detected.

+ `fifo_rate`

	> <=200Hz

#### rec_s

+ `float gyro[3]` 三轴角速度  \dps

+ `float accel[3]` 三轴加速度 \g

+ `long  quat[4]` dmp解算的四元数 \ 30位整型

+ `float pitch` `float roll` `float yaw` \degree

+ `short sensors`  数据对应的传感器标志

+ `unsigned char more`  剩余数据包

+ `unsigned long sensor_timestamp`  时间戳 \ms

### How to Init

默认开启陀螺仪和加速度计并将数据传入FIFO，采样率200Hz，加速度量程8G，角速度量程1000DPS，数字低通滤波上限截至频率98Hz，坐标系为芯片坐标系；开启dmp，（dmp设置为使用6轴数据解算四元数，角速度校准，dmp FIFO传入加速度和校准后的角速度，），dmp中断模式设置为DMP_INT_CONTINUOUS ，FIFO速率为200Hz。

若需要做修改，修改对应的结构体：`mpu_config` `dmp_config `即可

初始化代码：

```C++
MPU6050_Config_Pin(GPIOB,GPIO_PIN_6,GPIO_PIN_7);
MPU6050_Init(&mpu_config,&dmp_config,&mpu_receive);
MPU6050_run_self_test(false);//自检函数，参数对应是否将自检的加速度值作为偏差写入对应寄存器
```

### How to read data

如果开启了dmp：

```C++
dmp_read_data(&mpu_receive);
```

也可以直接读mpu数据：

```C++
mpu_read_data(&mpu_receive);
```

#### How to read data with interrupt

template:

```C++
const uint16_t MPU_INT  = GPIO_PIN_3;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	BaseType_t pxHigherPriorityTaskWoken;
	unsigned char dmp_state;
 	switch(GPIO_Pin){
	case GPIO_PIN_3:{
		mpu_get_dmp_state(&dmp_state);
		if(MPU_INT_semphr != NULL && dmp_state){
			xSemaphoreGiveFromISR(MPU_INT_semphr,&pxHigherPriorityTaskWoken);
		}
		break;
	}
	default: break;
	}
}
```

```C++
void tskMPU(void *arg){
	for(;;){
		if(xSemaphoreTake(MPU_INT_semphr,portMAX_DELAY) == pdPASS){
			dmp_read_data(&mpu_receive);
		}
	}
}
```

