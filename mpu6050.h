/*
 * mpu6050.h
 *
 *  Created on: Feb 5, 2026
 *      Author: nickzentmayer
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_

#include <stdio.h>
#include <stm32f4xx_hal.h>

#define MPU_DEBUG

//MPU6050 ADDRESS
#define MPU6050_ADR 0x68

//MPU6050 Registers
#define MPU_WHOAMI				0x75
#define MPU_USER_CTRL			0x6A
#define MPU_PWR_MGMT_1			0x6B
#define MPU_GYRO_CONFIG			0x1B
#define MPU_ACCEL_CONFIG		0x1C
#define MPU_CONFIG				0x1A
#define MPU_ACCEL_X_H			0x3B
#define MPU_ACCEL_X_L			0x3C
#define MPU_ACCEL_Y_H			0x3D
#define MPU_ACCEL_Y_L			0x3E
#define MPU_ACCEL_Z_H			0x3F
#define MPU_ACCEL_Z_L			0x40
#define MPU_TEMP_H				0x41
#define MPU_TEMP_L				0x42
#define MPU_GYRO_X_H			0x43
#define MPU_GYRO_X_L			0x44
#define MPU_GYRO_Y_H			0x45
#define MPU_GYRO_Y_L			0x46
#define MPU_GYRO_Z_H			0x47
#define MPU_GYRO_Z_L			0x48

//constants
#define G_TO_MS2 -9.8067
#define LSB_TO_G 16384.0
#define LSB_TO_DPS 131.0

//enumerations for gyroscope and accelerometer settings
typedef enum {
	MPU_ACCEL_RANGE_2G,
	MPU_ACCEL_RANGE_4G,
	MPU_ACCEL_RANGE_8G,
	MPU_ACCEL_RANGE_16G
}MPU_6050_ACCEL_CFG;

typedef enum {
	MPU_GYRO_RANGE_250DPS,
	MPU_GYRO_RANGE_500DPS,
	MPU_GYRO_RANGE_1000DPS,
	MPU_GYRO_RANGE_2000DPS
}MPU_6050_GYRO_CFG;

//data structure to hold each axis of (angular) acceleration
typedef struct {
	double x;
	double y;
	double z;
} mpu6050_3DData;


HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef* mpuI2C, MPU_6050_ACCEL_CFG accelScale, MPU_6050_GYRO_CFG gyroScale);

HAL_StatusTypeDef mpu6050_getAccelData(mpu6050_3DData* data);
HAL_StatusTypeDef mpu6050_getGyroData(mpu6050_3DData* data);
HAL_StatusTypeDef mpu6050_getRawAccelData(mpu6050_3DData* data);
HAL_StatusTypeDef mpu6050_getRawGyroData(mpu6050_3DData* data);
HAL_StatusTypeDef mpu6050_getTempData(double* data);


HAL_StatusTypeDef mpu6050_setDLPF(uint8_t dlpfSetting);
HAL_StatusTypeDef mpu6050_setAccelRange(MPU_6050_ACCEL_CFG accelScale);
HAL_StatusTypeDef mpu6050_setGyroRange(MPU_6050_GYRO_CFG gyroScale);



#endif /* SRC_MPU6050_H_ */
