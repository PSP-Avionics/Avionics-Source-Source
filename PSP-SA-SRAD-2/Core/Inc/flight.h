/*
 * flight.h
 *
 *  Created on: May 23, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_FLIGHT_H_
#define INC_FLIGHT_H_

#include <altimeter_old.h>
#include "settings.h"
#include "main.h"

#define FLIGHT_HEARTBEAT_INTERVAL 15
#define NEEDED_SAMPLES_FOR_VELOCITY 3


typedef enum {
	INIT,
	PAD,
	MOTOR_BURN,
	ASCENT,
	DESCENT,
	LANDED
} flight_state_t; // make sure to adjust flight_state_funcs as needed below

typedef enum {
	DROGUE, MAIN
} ejection_type_t;

typedef flight_state_t (flight_state_func_t)(system_data* data);

typedef struct _flight_data {
	double groundAlt; // ground altitude ASL
	double currAlt; // current altitude ASL
	double approxVelocity; // approximate velocity in feet/s ONLY IN THE UP DIRECTION
	uint8_t apogeeReached;
	flight_state_t state;
	uint8_t activeControlsPermitted;
	uint32_t lastUpdateMs;
} flight_data_t;


float interpolate_height_agl();
void init_flight(system_data* data);
void flight_log(char* message);
const flight_data_t flight_data();

// flight states

flight_state_t flight_state_init(system_data* data);
flight_state_t flight_state_pad(system_data* data);
flight_state_t flight_state_motor_burn(system_data* data);
flight_state_t flight_state_ascent(system_data* data);
flight_state_t flight_state_descent(system_data* data);
flight_state_t flight_state_landed(system_data* data);



#endif /* INC_FLIGHT_H_ */
