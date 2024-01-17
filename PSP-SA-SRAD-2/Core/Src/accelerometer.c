/*
 * accelerometer.c
 *
 *  Created on: Jun 26, 2022
 *      Author: Alex Bowman
 *
 *  Code for reading from accelerometer
 */


#include "main.h"
#include "accelerometer.h"
#include "lib/mpu6050.h"
#include "settings.h"

extern I2C_HandleTypeDef ACCELEROMETER_I2C_HANDLE; // trying to access the variable in "main.h"
mpu6050_data_t mpu;
vec3f rotation;

static uint32_t last_time_us;


void init_accelerometer(system_data* data) {
	heartbeat_entry* accel_entry = calloc(1, sizeof(heartbeat_entry));
	accel_entry->function = accelerometer_heartbeat;
	accel_entry->interval = ACCELEROMETER_HEARTBEAT_INTERVAL;
	accel_entry->next = null;
	accel_entry->timeUntilNext = 0; // give 3ms so hopefully tasks dont overlap, plus 150 MS so that the BME280 device can take at least one sample
	accel_entry->name = "mpu6050";

	register_heartbeat_func(accel_entry);

	mpu6050_init(&ACCELEROMETER_I2C_HANDLE);

	last_time_us = get_time_us();

	accelerometer_heartbeat(data); // read data
}


static void accelerometer_heartbeat(system_data* data) {
	mpu6050_read_accel(&ACCELEROMETER_I2C_HANDLE, &mpu);
	mpu6050_read_gyro(&ACCELEROMETER_I2C_HANDLE, &mpu);

	uint32_t timeNow = get_time_us();
	uint32_t timeDiff = timeNow - last_time_us;


	if (abs((int) mpu.Gx) >= 2) {
		rotation.x += ((double) mpu.Gx) * timeDiff / 1000000;
	}
	if (abs((int) mpu.Gy) >= 2) {
		rotation.y += ((double) mpu.Gy) * timeDiff / 1000000;
	}
	if (abs((int) mpu.Gz) >= 2) {
		rotation.z += ((double) mpu.Gz) * timeDiff / 1000000;


//		char* message = smintf("GYRO z %d %d %f", timeDiff, get_time_us(), (float) mpu.Gx);
//		log_message(message);
//		free(message);
	}

	last_time_us = timeNow;

}

void accel_acceleration(vec3f* ptr) {
	ptr->x = mpu.Ax;
	ptr->y = mpu.Ay;
	ptr->z = mpu.Az;
}

void accel_gyro(vec3f* ptr) {
//	ptr->x = mpu.Gx;
//	ptr->y = mpu.Gy;
//	ptr->z = mpu.Gz;
	ptr->x = rotation.x;
	ptr->y = rotation.y;
	ptr->z = rotation.z;
}
