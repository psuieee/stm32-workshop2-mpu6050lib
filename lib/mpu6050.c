#include "mpu6050.h"

I2C_HandleTypeDef* MPU6050_I2C_Handler;
uint8_t aScale; //Save scale passed in init for conversion later
uint8_t gScale;	//Save scale passed in init for conversion later

//library functions
HAL_StatusTypeDef mpu6050_readByte(uint8_t address, uint8_t* data);
HAL_StatusTypeDef mpu6050_readBytes(uint8_t address, uint8_t* data, uint8_t length);
HAL_StatusTypeDef mpu6050_writeByte(uint8_t address, uint8_t data);

/*	mpu6050_init()
 * 	Returns - any error thrown by i2c HAL or HAL_OK if no errors occurred
 * 	In - STM32 I2C HAL handler, number from 0-3 inclusive to select accelerometer measurement range (full range = 2^accelScale)
 * 	Stores the I2C Handle for other functions to make the other calls easier
 */

HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef* mpuI2C, MPU_6050_ACCEL_CFG accelScale, MPU_6050_GYRO_CFG gyroScale) {
	//save i2c handler for later
	MPU6050_I2C_Handler = mpuI2C;
	
	uint8_t data; //place to put return data of read/write
	
	//first wake up MPU6050 if asleep
	HAL_StatusTypeDef status = mpu6050_writeByte(MPU_PWR_MGMT_1, 0x02);
	if (status != HAL_OK)
		return status;
	
	//lets check the whoami register to make sure we can communicate
	status = mpu6050_readByte(MPU_WHOAMI, &data);

	#ifdef MPU_DEBUG
		printf("WHOAMI: %x\n", data);
	#endif

	if (status != HAL_OK)
		return status;

	data = 0x45; //check datasheet for value meaning
	status = mpu6050_writeByte(MPU_USER_CTRL, data);
	if (status != HAL_OK)
			return status;

	//ACCEL_CONFIG
	data = (0x03 & accelScale) << 3; //accelScale config bits are [4:3], mask the input and shift accordingly
	aScale = accelScale;
	status = mpu6050_writeByte(MPU_ACCEL_CONFIG, data);
	if (status != HAL_OK)
			return status;

	//GYRO_CONFIG
	data = (0x03 & gyroScale) << 3; //accelScale config bits are [4:3], mask the input and shift accordingly
	//data |= 0x80;
	gScale = gyroScale;
	status = mpu6050_writeByte(MPU_GYRO_CONFIG, data);
	
	return status;
}

/*getAccelData
* Description: Read accelerometer data from the MPU6050, then converts to m/s^2
* In: mpu6050_3DData data structure pointer to store 3 axis of accelerometer data
* Returns: HAL_StatusTypeDef returned from I2C HAL
*/
HAL_StatusTypeDef mpu6050_getAccelData(mpu6050_3DData* data) {
	uint8_t readData[6];

	//X High byte
	HAL_StatusTypeDef status = mpu6050_readBytes(MPU_ACCEL_X_H, readData, 6);
	if (status != HAL_OK)
		return status;
	//here we turn 2 unsigned 8-bit values returned from the MPU6050 and covert
	//them to a single signed 16-bit integer, then save in a double type variable
	//repeat for each axis
	data->x = (int16_t)(((uint16_t)readData[0] << 8) | readData[1]);
	data->y = (int16_t)(((uint16_t)readData[2] << 8) | readData[3]);
	data->z = (int16_t)(((uint16_t)readData[4] << 8) | readData[5]);


	//convert to Gs
	data->x /= LSB_TO_G/(aScale + 1);
	data->y /= LSB_TO_G/(aScale + 1);
	data->z /= LSB_TO_G/(aScale + 1);

	//convert to m/s^2
	data->x *= G_TO_MS2;
	data->y *= G_TO_MS2;
	data->z *= G_TO_MS2;

	return status;
}

/*getGyroData
* Description: Read gyroscope data from the MPU6050, then converts to degrees/s^2
* In: mpu6050_3DData data structure pointer to store 3 axis of gyroscope data
* Returns: HAL_StatusTypeDef returned from I2C HAL
*/
HAL_StatusTypeDef mpu6050_getGyroData(mpu6050_3DData* data) {
	uint8_t readData[6];

	//Registers are sequential, start at X_HIGH register
	HAL_StatusTypeDef status = mpu6050_readBytes(MPU_GYRO_X_H, readData, 6);
	if (status != HAL_OK)
		return status;
	//here we turn 2 unsigned 8-bit values returned from the MPU6050 and covert
	//them to a single signed 16-bit integer, then save in a double type variable
	//repeat for each axis
	data->x = (int16_t)(((uint16_t)readData[0] << 8) | readData[1]);
	data->y = (int16_t)(((uint16_t)readData[2] << 8) | readData[3]);
	data->z = (int16_t)(((uint16_t)readData[4] << 8) | readData[5]);


	//convert to degrees per second
	data->x /= LSB_TO_DPS/(gScale + 1);
	data->y /= LSB_TO_DPS/(gScale + 1);
	data->z /= LSB_TO_DPS/(gScale + 1);

	return status;
}

/*getAccelData
* Description: Read accelerometer data from the MPU6050, skips conversion
* In: mpu6050_3DData data structure pointer to store 3 axis of accelerometer data
* Returns: HAL_StatusTypeDef returned from I2C HAL
*/
HAL_StatusTypeDef mpu6050_getRawAccelData(mpu6050_3DData* data) {
	uint8_t readData[6];

	//X High byte
	HAL_StatusTypeDef status = mpu6050_readBytes(MPU_ACCEL_X_H, readData, 6);
	if (status != HAL_OK)
		return status;
	//here we turn 2 unsigned 8-bit values returned from the MPU6050 and covert
	//them to a single signed 16-bit integer, then save in a double type variable
	//repeat for each axis
	data->x = (int16_t)(((uint16_t)readData[0] << 8) | readData[1]);
	data->y = (int16_t)(((uint16_t)readData[2] << 8) | readData[3]);
	data->z = (int16_t)(((uint16_t)readData[4] << 8) | readData[5]);

	return status;
}

/*getRawGyroData
* Description: Read gyroscope data from the MPU6050, skips conversion
* In: mpu6050_3DData data structure pointer to store 3 axis of gyroscope data
* Returns: HAL_StatusTypeDef returned from I2C HAL
*/
HAL_StatusTypeDef mpu6050_getRawGyroData(mpu6050_3DData* data) {
	uint8_t readData[6];

	//Registers are sequential, start at X_HIGH register
	HAL_StatusTypeDef status = mpu6050_readBytes(MPU_GYRO_X_H, readData, 6);
	if (status != HAL_OK)
		return status;
	//here we turn 2 unsigned 8-bit values returned from the MPU6050 and covert
	//them to a single signed 16-bit integer, then save in a double type variable
	//repeat for each axis
	data->x = (int16_t)(((uint16_t)readData[0] << 8) | readData[1]);
	data->y = (int16_t)(((uint16_t)readData[2] << 8) | readData[3]);
	data->z = (int16_t)(((uint16_t)readData[4] << 8) | readData[5]);

	return status;
}

/*getTempData
* Description: Read temperature data from the MPU6050
* In: double pointer to store temperature data, in Celsius
* Returns: HAL_StatusTypeDef returned from I2C HAL
*/
HAL_StatusTypeDef mpu6050_getTempData(double* data) {
	uint8_t readData[2];

	//Registers are sequential, start at X_HIGH register
	HAL_StatusTypeDef status = mpu6050_readBytes(MPU_TEMP_H, readData, 2);
	if (status != HAL_OK)
		return status;
	//here we turn 2 unsigned 8-bit values returned from the MPU6050 and covert
	//them to a single signed 16-bit integer, then save in a double type variable
	*data = (int16_t)(((uint16_t)readData[0] << 8) | readData[1]);

	//now convert to degrees Celsius
	*data = (*data / 340.0) + 36.53;

	return status;
}


/*
 * setDLPF
 * Description: sets the digital low pass filter's configuration bits, changes MPU's update rate
 * In: uint8_t from 0-7 inclusive for the DLPF setting
 * Out: HAL_StatusTypeDef returned from I2C HAL
 */
HAL_StatusTypeDef mpu6050_setDLPF(uint8_t dlpfSetting) {
	return mpu6050_writeByte(MPU_CONFIG, dlpfSetting & 0x07);
}

/*
 * setAccelRange
 * Description: sets the accelerometer's configuration bits, changes MPU's sensitivity
 * In: MPU_6050_ACCEL_CFG accelScale to set range, saves input for read calculations
 * Out: HAL_StatusTypeDef returned from I2C HAL
 */
HAL_StatusTypeDef mpu6050_setAccelRange(MPU_6050_ACCEL_CFG accelScale) {
	aScale = accelScale;
	return mpu6050_writeByte(MPU_ACCEL_CONFIG, (0x03 & accelScale) << 3);
}

/*
 * setGyroRange
 * Description: sets the gyroscope's configuration bits, changes MPU's sensitivity
 * In: MPU_6050_GYRO_CFG gyroScale to set range, saves input for read calculations
 * Out: HAL_StatusTypeDef returned from I2C HAL
 */
HAL_StatusTypeDef mpu6050_setGyroRange(MPU_6050_GYRO_CFG gyroScale) {
	gScale = gyroScale;
	return mpu6050_writeByte(MPU_GYRO_CONFIG, (0x03 & gyroScale) << 3);
}


//private functions

HAL_StatusTypeDef mpu6050_readByte(uint8_t address, uint8_t* data) {

	return HAL_I2C_Mem_Read(MPU6050_I2C_Handler, (MPU6050_ADR << 1),
			address, 1, data, 1, 1000);
			
}

HAL_StatusTypeDef mpu6050_readBytes(uint8_t address, uint8_t* data, uint8_t length) {

	return HAL_I2C_Mem_Read(MPU6050_I2C_Handler, (MPU6050_ADR << 1),
			address, 1, data, length, 1000);

}

HAL_StatusTypeDef mpu6050_writeByte(uint8_t address, uint8_t data) {

	return HAL_I2C_Mem_Write(MPU6050_I2C_Handler,
	 					(MPU6050_ADR << 1),
						address,
						1,
						&data,
						1,
						1000);
						
}
