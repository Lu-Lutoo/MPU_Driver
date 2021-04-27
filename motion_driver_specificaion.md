# motion driver specificaion

[TOC]

看看官方文档的时候记一下笔记


## files
+ inv_mpu.c	|	inv_mpu.h
+ inv_mpu_dmp_motion_driver.c	|	inv_mpu_dmp_motion_driver.h
## applicaions
+ 按给定模式初始化MPU
+ 轮询模式读取mpu存取测量数据的FIFO
+ 轮询模式读取dmp存取测量数据和解算数据的FIFO
+ 中断模式读取dmp存取测量数据和解算数据的FIFO
+ DMP calibration
+ 按照设定的阈值中断形式检测运动

- How to load, configure, and leverage DMP functions
- Gyroscope and accelerometer self-test function calls based on the hardware self-test document
- Accelerometer calibration and updating parameters within the hardware registers
- Gyroscope calibration
- Configuring the low power accelerometer motion interrupt
- Ability to change the sensor Output Data Rate (ODR) of the gyroscope and accelerometer
- Ability to select which data is populated in the FIFO


## functions

1. DMP and DMP features:
    dmp为invense mpu 的数字运动处理器，它可以解算四元素。dmp映像存储在mpu的非永久存储器,所以每次上电都要重新设置一遍
    
    函数：`dmp_enable_feature`
    
    (以下不是函数参数)
    
    + **`DMP_FEATURE_LP_QUAT`** 200Hz，由gyro解算四元素，低功耗。
    + **`DMP_FEATURE_6X_LP_QUAT`** 200Hz，由gyro和accel解算四元素，低功耗。
    + **`DMP_FEATURE_TAP`**  识别敲击事件并分辨其基本的特征，比如单击、双击，或者敲击的方向。
    + **`DMP_FEATURE_ANDROID_ORIENT`** 此功能是为了兼容谷歌Motion Driver设备
    + **`DMP_PEDOMETER`** 计步特性，一直使能
    + **`DMP_FEATURE_GYRO_CAL`** 每次设备静止超过8秒时，校准陀螺仪零偏。
    + **`DMP_FEATURE_SEND_RAW_ACCEL`** 将加速度计的raw轴（偏航）的数据放入FIFO，此数据基于芯片坐标系。
    + **`DMP_FEATURE_SEND_RAW_GYRO`** 将陀螺仪的raw轴（偏航）的数据放入FIFO，此数据基于芯片坐标系。
    + **`DMP_FEATURE_SEND_CAL_GYRO`** 将陀螺仪的校准后的数据放入FIFO。不可与DMP_FEATURE_SEND_RAW_GYRO结合使用。输出不基于芯片坐标系，而是基于设备坐标系
    
2. Steps to load and enable DMP features

    1.  传送dmp映像给mpu内存 dmp_load_motion_driver_firmware
    2.  推送陀螺仪和加速度计的方向矩阵到DMP dmp_set_orientation 
    3.  给部分功能提供回调函数
    4.  使能DMP特性  dmp_enable_feature
    5.  设置fifo速率
    
3. 陀螺仪和加速度计自测(self_test)
    
    通过函数`mpu_run_self_test`自检，然后利用返回的值计算陀螺仪基准值和加速度计基准值，再使用函数
    `mpu_set_gyro_bias_reg`和`mpu_set_accel_bias_6050_reg`写入基准值。
    自检的时候确保imu放在一个水平面上。
    .e.g
    ``` C
    void run_self_test(void)
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
    ``` 
    
4. 低功耗加速度计中断模式
    就是设置一个加速度阈值，加速度计存储一次加速度数据后，继续读取之后的数据，一旦有一次数据与开始记录的数据的差大于阈值，则触发中断，可以用在运动检测上
    
5. 输出速率可改变的加速度计和陀螺仪
    InvenSense MPU 提供了可编程的陀螺仪、加速度计数据输出速率范围（ODR）。该范围可通过写值到SMPLRT_DIV配置，陀螺仪的输出数据速率计算公式如下
    > `Sample Rate = Gyroscope Output Rate /(1+SMPLRT_DIV)`
    
    Motion Driver 提供了一个简单的方法配置陀螺仪的ODR——调用函数mpu_set_sample_rate。不过，当DMP打开，陀螺仪就预设为200Hz采样率，并不需要调用mpu_set_sample_rate。当DMP关闭，最大值可以设置为最高8Khz，取决于MPU设备特性。当DLPF失能（DLPF_CFG=0 or 7）最大陀螺仪输出速率为8Khz，当DLPF使能时为1Khz。最大加速度计输出速率为1Khz。当采样率高于1KHz时，加速度计的同一次采样数据可能会在FIFO、DMP和传感器寄存器中输出超过一次
    
6. 可以设置什么数据才有资格传到FIFO
    `mpu_configure_fifo(unsigned char sensors)`
    


## Applications

 1. ==Init==
 
        函数: `mpu_init`
        函数: `mpu_set_dmp_state` 启动dmp
        函数: `mpu_set_sensors`  选择开启的传感器
        函数: `mpu_run_self_test`
        函数: `mpu_load_firmware`

 1. ==dmp interrupt==
    
        函数: `dmp_set_interrupt_mode`
        函数: `mpu_set_int_level` 有滴唔太明
        
    有两种模式供dmp中断。一种是dmp的FIFO完成一次数据存储，另一种是敲击事件。另外，mpu可以玩敲击检测，步数计数，跑步时间计时等，~~但是我不想写 :joy:~~ 后续版本加入 。
 2. ==FSR==
 
    大概有几个速率可以改:
    * *mpu_sample_rate* :    <br>   `mpu_get_sample_rate`   |  `mpu_set_sample_rate`
    * *dmp_fifo_rate*:       <br>   `dmp_get_fifo_rate`     |  `dmp_set_fifo_rate`
    * *compass_rate*:        <br>   `mpu_get_compass_rate`  |  `mpu_set_compass_sample_rate`
    
 3. ==calibration==
 
        函数： `dmp_enable_gyro_cal`
        函数: `mpu_set_accel_bias`
 
    8秒静止后，dmp会计算陀螺仪偏置然后从四元素中减去(为什么是从四元素减？ :s)，如果dmp_enable_feature是DMP_FEATURE_-
SEND_CAL_GYRO,那么角速度也会减去偏置。
 
 4. ==FIFO==
 
        函数: `dmp_read_fifo` 
        函数: `mpu_read_fifo`
        函数: `mpu_configure_fifo` 
        函数: `mpu_get_fifo_config`
 
 5. ==temperature==
    
        函数: `mpu_get_tempetature`
 6. == filter == 
 
        函数: `mpu_set_lpf` 设置低通滤波
 
 ## 杂项
 1. 使能6_axis DMP解算四元素（低功耗）  <br>`dmp_enable_6x_lp_quat`
 2. 使能3_axis DMP解算四元素（低功耗）  <br>`dmp_enable_lp_quat`
 3. 使能运动检测                        <br>`dmp_enable_no_motion_detection` <br>  <font size = 2 color = purple>*dmp文件里没有看到这个函数*</font>
 4. 步数计数                            <br>`dmp_get_pedometer_step_count`
 5. 走路计时                            <br>`dmp_get_pedometer_walk_time`
 6. 注册运动检测回调函数                <br>`dmp_register_no_motion_cb`
 7. 注册敲击检测回调函数                <br>`dmp_register_tap_cb`
 8. 设置dmp加速度偏置                   <br>`dmp_set_accel_bias`
 9. 设置dmp角速度偏置                   <br>`dmp_set_gyro_bias`
 10. 设置运动检测阈值                   <br>`dmp_set_no_motion_thresh`
 11. 设置运动检测触发报告最短静止时间   <br>`dmp_set_no_motion_time`
 12. 设置dmp方向（输入方向矩阵）        <br>`dmp_set_orientation`
 13. 覆盖写步数                         <br>`dmp_set_pedometer_step_count`
 14. 覆盖写走路时间                     <br>`dmp_set_pedometer_walk_time`
 15. 设置摇动检测阈值                   <br>`dmp_set_shake_reject_thresh`
 16. 设置摇动检测触发报告的最短摇动时间 <br>`dmp_set_shake_reject_time`
 17. 设置摇动检测触发报告的最长摇动时间 <br>`dmp_set_shake_reject_time`
 18. 设置敲击检测方向                   <br>`dmp_set_tap_axes`
 19. 设置触发中断的最少敲击次数         <br>`dmp_set_tap_count`
 20. 设置敲击检测阈值                   <br>`dmp_set_tap_thresh`
 21. 设置有效敲击的最短间隔时间         <br>`dmp_set_tap_time`
 22. 设置多重敲击中每个敲击的最长间隔   <br>`dmp_set_tap_time_multi`
 23. 直接从寄存器读取加速度数据         <br>`mpu_get_accel_reg`<br> <font size = 2 color =  purple>所以暗示加速度是会被处理的？ </font> :upside_down_face:
 24. 获取加速度灵敏值                   <br>`mpu_get_accel_sens`
 25. 直接从寄存器读取磁力计数据         <br>`mpu_get_compass_reg`
 26. 获取陀螺仪频率？？？               <br>`mpu_get_gyro_fsr`
 27. 获取加速度计频率？？               <br>`mpu_set_accel_fsr`
 27. 直接从寄存器读取陀螺仪数据         <br>`mpu_get_gyro_reg`
 28. 获取陀螺仪灵敏值                   <br>`mpu_get_gyro_sens`<br> <font size = 2 color = purple>我没搞懂灵敏值拿来干嘛</font> :rofl:
 29. 获取mpu中断状态                    <br>`mpu_get_int_status`
 30. 获取mpu的DLPF设置                  <br>`mpu_get_lpf` <font size = 2 color = purple> DLPF是啥？？？</font> :rofl:
 31. 获取mpu工作状态(正常/休眠)         <br>`mpu_get_power_state`
 32. 进入低功耗仅获取加速度模式         <br>`mpu_lp_accel_mode`
 33. 进入低功耗中断模式                 <br>`mpu_lp_motion_interrupt`<br><font size = 2 color = purple> 看函数描述觉得它和运动检测很像，后面分析区别 </font>
 34. 重置mpu的FIFO                      <br>`mpu_reset_fifo`
 
  ~~太底层的我就懒得记了~~  ==后面是底层的函数==
  
 35. `mpu_read_fifo_stream`<br>`mpu_read_mem`<br>`mpu_read_reg`<br>`mpu_reg_dump`<br>`mpu_write_mem` <font size = 1 >写dmp memory</font>
 
 ==后面的函数==~~都不知道什么鬼~~==我不太懂==
 
 36. mpu模式设置为旁路                  <br>`mpu_set_bypass`
 37. 设置陀螺仪频率                     <br>`mpu_set_gyro_fsr`
 38. 不知道开了什么中断                 <br>`mpu_set_int_latched`