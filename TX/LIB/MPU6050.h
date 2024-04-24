#ifndef MPU6050_H
#define MPU6050_H
#include "stm32f1xx_hal.h"

#define DEVICE_ADDRESS 0x68

#define FS_GYRO_250      0
#define FS_GYRO_500      8
#define FS_GYRO_1000     9
#define FS_GYRO_2000     10

#define FS_ACC_2G        0
#define FS_ACC_4G        8
#define FS_ACC_8G        9
#define FS_ACC_16G       10

#define REG_CONFIG_GYRO       27
#define REG_CONFIG_ACC        28
#define REG_USR_CTRL          107
#define REG_DATA_Xacc              59
#define REG_DATA_Yacc              61

void mpu6050_init(void);

int16_t mpu6050_read_x(void);
int16_t mpu6050_read_y(void);




#endif

