/*
 * mpu6050.c
 *
 *  Created on: July 2, 2021
 *      Author: Alex Bowman
 *
 *  Using code from
 *  	https://github.com/RullyanGS/Nucleo-L476RG-mpu6050-library as well as
 *  	https://github.com/right13/STM32-Nucleo/blob/master/MPU6050 for reference
 */

#include <math.h>

#include "lib/mpu6050.h"
#include "main.h"
#include "assert.h"

#define MPU6050_ADDR 0x68<<1

#define RA_WHO_AM_I         0x75
#define RA_CONFIG           0x1A
#define RA_USER_CTRL        0x6A
#define RA_PWR_ADDR   	    0x6B
#define RA_SMPLRT_DIV	    0x19
#define RA_GYRO_CONFIG      0x1B
#define RA_ACCEL_CONFIG     0x1C
#define RA_INT_PIN_CFG      0x37
#define RA_ACCEL_XOUT_H     0x3B
#define RA_ACCEL_XOUT_L     0x3C
#define RA_ACCEL_YOUT_H     0x3D
#define RA_ACCEL_YOUT_L     0x3E
#define RA_ACCEL_ZOUT_H     0x3F
#define RA_ACCEL_ZOUT_L     0x40
#define RA_TEMP_OUT_H       0x41
#define RA_TEMP_OUT_L       0x42
#define RA_GYRO_XOUT_H      0x43
#define RA_GYRO_XOUT_L      0x44
#define RA_GYRO_YOUT_H      0x45
#define RA_GYRO_YOUT_L      0x46
#define RA_GYRO_ZOUT_H      0x47
#define RA_GYRO_ZOUT_L      0x48
#define AF_SEL 							0x00
#define ACC_HPF 						0x00
#define FS_SEL 							0x00
#define RA_PWR_MGMT_1  			0x00
#define USER_CTRL        		0x00
#define ACC_SELF_TEST_X 		0x00
#define ACC_SELF_TEST_Y		 	0x00
#define ACC_SELF_TEST_Z 		0x00
#define G_SELF_TEST_X 			0x00
#define G_SELF_TEST_Y 			0x00
#define G_SELF_TEST_Z 			0x00
#define PIN_CFG							2
#define CONFIG							4
#define SMPLRT_DIV					19

static void i2c_write(I2C_HandleTypeDef* dev, uint8_t subAddr, uint8_t data);
static int16_t i2c_read(I2C_HandleTypeDef* dev, uint8_t addr_L, uint8_t addr_H);

static uint8_t isDisabled;

void mpu6050_init (I2C_HandleTypeDef* dev) {

	i2c_write(dev, RA_PWR_ADDR, RA_PWR_MGMT_1);
	HAL_Delay(100);
	i2c_write(dev, RA_USER_CTRL, USER_CTRL);
	i2c_write(dev, RA_INT_PIN_CFG, PIN_CFG);
	i2c_write(dev, RA_SMPLRT_DIV, SMPLRT_DIV);
	i2c_write(dev, RA_GYRO_CONFIG, G_SELF_TEST_X | G_SELF_TEST_Y | G_SELF_TEST_Z | FS_SEL);
	i2c_write(dev, RA_ACCEL_CONFIG, ACC_SELF_TEST_X | ACC_SELF_TEST_Y | ACC_SELF_TEST_Z | AF_SEL | ACC_HPF);

	uint8_t whoAmI = 0x75;
	HAL_I2C_Master_Transmit(dev, MPU6050_ADDR, &whoAmI, 1, HAL_MAX_DELAY);
	whoAmI = 0;
	HAL_I2C_Master_Receive(dev, MPU6050_ADDR, &whoAmI, 1, HAL_MAX_DELAY);

	char* message = smintf("mpu6050 returned %d, not 114", (int) whoAmI);
	log_message(message);
	free(message);
//	assert(0);

	if (whoAmI != 114) {
		isDisabled = 1;
	}


}

void mpu6050_read_accel (I2C_HandleTypeDef* dev, mpu6050_data_t* out_data)
{
	if (isDisabled) {
		out_data->Ax = NAN;
		out_data->Ay = NAN;
		out_data->Az = NAN;
		return;
	}

	int16_t data[3];

	data[0] = i2c_read(dev, RA_ACCEL_XOUT_L, RA_ACCEL_XOUT_H);
	data[1] = i2c_read(dev, RA_ACCEL_YOUT_L, RA_ACCEL_YOUT_H);
	data[2] = i2c_read(dev, RA_ACCEL_ZOUT_L, RA_ACCEL_ZOUT_H);


	//see the other github for the reasons behind these numbers
	out_data->Ax = data[0]/16384.0;
	out_data->Ay = data[1]/16384.0;
	out_data->Az = data[2]/16384.0;
}

void mpu6050_read_gyro (I2C_HandleTypeDef* dev, mpu6050_data_t* out_data)
{
	if (isDisabled) {
		out_data->Gx = NAN;
		out_data->Gy = NAN;
		out_data->Gz = NAN;
		return;
	}

	int16_t data[3];

	data[0] = i2c_read(dev, RA_GYRO_XOUT_L, RA_GYRO_XOUT_H);
	data[1] = i2c_read(dev, RA_GYRO_YOUT_L, RA_GYRO_YOUT_H);
	data[2] = i2c_read(dev, RA_GYRO_ZOUT_L, RA_GYRO_ZOUT_H);

//	log_messagef("Gyro raw data: %d %d %d", (int) data[0], (int) data[1], (int) data[2]);

	// see the other github for the reasons behind these numbers
	out_data->Gx = data[0]/131.0;
	out_data->Gy = data[1]/131.0;
	out_data->Gz = data[2]/131.0;
}


static void i2c_write(I2C_HandleTypeDef* dev, uint8_t subAddr, uint8_t data) {
	if (isDisabled)
		return;

	char buf[2];
	buf[0] = subAddr;
	buf[1] = data;
	HAL_I2C_Master_Transmit(dev, MPU6050_ADDR, &buf, 2, 5);
}

static int16_t i2c_read(I2C_HandleTypeDef* dev, uint8_t addr_L, uint8_t addr_H) {
	if (isDisabled)
		return;

	uint8_t data[2];
	HAL_I2C_Master_Transmit(dev, MPU6050_ADDR, &addr_L, 1, 5);
	HAL_I2C_Master_Receive(dev, MPU6050_ADDR, &(data[0]), 1, 5);
	HAL_I2C_Master_Transmit(dev, MPU6050_ADDR, &addr_H, 1, 5);
	HAL_I2C_Master_Receive(dev, MPU6050_ADDR, &(data[1]), 1, 5);

	return (uint16_t) (((uint16_t) data[1]) << 8) | data[0];

}
