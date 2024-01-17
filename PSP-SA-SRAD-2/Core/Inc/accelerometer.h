/*
 * accelerometer.h
 *
 *  Created on: Jun 26, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "main.h"
#include <stdint.h>


#define ACCELEROMETER_HEARTBEAT_INTERVAL 2 // 500Hz

static void accelerometer_heartbeat(system_data* data);
void init_accelerometer(system_data* data);
void accel_acceleration(vec3f* ptr); // returns the acceleration
void accel_gyro(vec3f* ptr); // returns the gyro direction


#endif /* INC_ACCELEROMETER_H_ */
