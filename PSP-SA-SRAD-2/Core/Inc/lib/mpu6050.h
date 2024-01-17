/*
 * mpu6050.h
 *
 *  Created on: July 2, 2021
 *      Author: Alexander Bowman
 *
 *  Using code from https://github.com/RullyanGS/Nucleo-L476RG-mpu6050-library
 *  for reference
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#endif /* INC_MPU6050_H_ */

#include <stdint.h>
#include "main.h"

//MPU6050 structure
typedef struct {

	float Ax;
	float Ay;
	float Az;

	float Gx;
	float Gy;
	float Gz;

} mpu6050_data_t;

void mpu6050_init (I2C_HandleTypeDef* i2c_dev);
void mpu6050_read_accel (I2C_HandleTypeDef* i2c_dev, mpu6050_data_t* data);
void mpu6050_read_gyro (I2C_HandleTypeDef* i2c_dev, mpu6050_data_t* data);
