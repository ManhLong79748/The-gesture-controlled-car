#include "MPU6050.h"

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init()
{
	
		HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS <<1)+0,1,100);
	
		uint8_t temp_data = FS_GYRO_500; 
		HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS <<1)+0,REG_CONFIG_GYRO,1,&temp_data,1,100 );
	
		temp_data = FS_ACC_4G; 
		HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS <<1)+0,REG_CONFIG_ACC,1,&temp_data,1,100 );	
	
		temp_data = 0; 
		HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS <<1)+0,REG_USR_CTRL,1,&temp_data,1,100 );	
} 

int16_t mpu6050_read_x()
{
	uint8_t data[2];
	int16_t x_acc;
	HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS <<1)+1,REG_DATA_Xacc,1,data,2 ,100);
	x_acc = ((int16_t)data[0] << 8) + data[1];
	return x_acc;
}
int16_t mpu6050_read_y()
{
	uint8_t data[2];
	int16_t y_acc;
	HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS <<1)+1,REG_DATA_Yacc,1,data,2 ,100);
	y_acc = ((int16_t)data[0] << 8) + data[1];
	return y_acc;
}

































