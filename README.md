# MPU_Driver

## MPU_6_axis Library

![](https://img.shields.io/badge/version-V1.0-blue.svg)
![](https://img.shields.io/badge/state-pass%20test-green)

### 开发历程

开始傻逼地以为可以写一个库兼容mpu6050和mpu9250，我太naive了，mpu9250就是mpu6500+磁力计，然后mpu6050也可以搭配磁力计使用，同时mpu6050与mpu6500性能上差距较小（从官方数据上看mpu6050比mpu6500优），所以问题就变成6_axisIMU+磁力计与6_axisIMU的代码有多大区别。

9_axisMPU代码大概可以分三类：直接读取测量数据、dmp解算出四元素、mpl算法解算。如果你不是自己写解算代码，而是使用dmp或mpl，那么6_axisMPU的代码和9_axisMPU代码的解算部分没有任何相同的地方，不如我直接分两个库。

所以我计划先使用官方的motion_driver_5.1.3完成mpu6050库，然后再兼容mpu6500。

不过最近发现github上jrowberg为主的人开发了一个i2c库，并提供了神奇的mpu6050代码，之后可以了解。

2021.4.28 赶出第一份未测试的mpu6050二次封装代码，一些注释没有完成，使用说明也没有写，~~不想写~~有点累

2021.6.18 解决完mpu6050二次封装代码的所有bug

### 介绍

请移步`mpu6050_driver_lib.md`