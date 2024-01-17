/*
 * flight.c
 *
 *  Created on: May 23, 2022
 *      Author: Alex Bowman
 *
 *  Code for managing the flight of the rocket (parachute deployment, detecting liftoff, etc.)
 */

#include <altimeter_old.h>
#include "main.h"
#include "fatfs.h"
#include "lib/bme280.h"
#include "flight.h"
#include "audio.h"
#include "fs_storage.h"
#include <math.h>
#include <stdbool.h>

static void flight_heartbeat(system_data* data);
static FIL* f_flight_log;

static flight_data_t flightData = {
		.groundAlt = -1,
		.approxVelocity = -1,
		.apogeeReached = 0,
		.state = INIT
};

const flight_state_func_t* flight_state_funcs[] = {
	flight_state_init,
	flight_state_pad,
	flight_state_motor_burn,
	flight_state_ascent,
	flight_state_descent,
	flight_state_landed
};

static double _last_heights[NEEDED_SAMPLES_FOR_VELOCITY+1];

void init_flight(system_data* data) {
	heartbeat_entry* flight_entry = calloc(1, sizeof(heartbeat_entry));
	flight_entry->function = flight_heartbeat;
	flight_entry->interval = FLIGHT_HEARTBEAT_INTERVAL;
	flight_entry->next = null;
	flight_entry->timeUntilNext = 153; // give 3ms so hopefully tasks dont overlap, plus 150 MS so that the BME280 device can take at least one sample
	flight_entry->name = "flight";

	register_heartbeat_func(flight_entry);

	f_flight_log = (FIL*) calloc(1, sizeof(FIL));
	f_open(f_flight_log, "/flight/flightlog.txt", FA_CREATE_ALWAYS | FA_WRITE);

	flight_log("Flight Initialized, assuming all systems are GO\n");

	for (uint8_t i = 0; i < NEEDED_SAMPLES_FOR_VELOCITY + 1; i ++) {
		_last_heights[i] = -1;
	}
}

void flight_heartbeat(system_data* data) {
	if (flightData.groundAlt == -1) {
		struct bme280_data thp_data;
		flightData.groundAlt = get_altitude_feet();

		char* message = smintf("Detected ground level: %f ft\n", (float) flightData.groundAlt);
		flight_log(message);
		free(message);
	}

	// get new altitude

	flightData.currAlt = get_altitude_feet();

	// calculate velocity
	for (int8_t i = NEEDED_SAMPLES_FOR_VELOCITY; i > 0; i --) {
		_last_heights[i] = _last_heights[i-1];
	}
	_last_heights[0] = flightData.currAlt;

	double vel = 0;
	for (int8_t i = 0; i < NEEDED_SAMPLES_FOR_VELOCITY; i ++) {
		vel += (_last_heights[i] - _last_heights[i+1]) * 1000 / FLIGHT_HEARTBEAT_INTERVAL;
		if (_last_heights[i+1] == -1) vel = -NEEDED_SAMPLES_FOR_VELOCITY;
	}
	vel /= NEEDED_SAMPLES_FOR_VELOCITY;

	flightData.approxVelocity = vel;
	flightData.lastUpdateMs = HAL_GetTick();

	// do things depending on flight state

	flightData.state = flight_state_funcs[flightData.state](data);
}


flight_state_t flight_state_init(system_data* data) {
	return PAD;
}

uint32_t _fsp_last_time = 0;

flight_state_t flight_state_pad(system_data* data) {
	if (HAL_GetTick() / 15000 > _fsp_last_time) {
		_fsp_last_time = HAL_GetTick() / 15000;
		morse_code("ready");
	}


	if (flightData.approxVelocity == -1) return PAD;
	if (flightData.approxVelocity < 20) return PAD;
	if (flightData.currAlt < flightData.groundAlt + 100) return PAD;

	char* message = smintf("Detected liftoff. Approximate velocity: %f", (float) flightData.approxVelocity);
	flight_log(message);
	free(message);

	return MOTOR_BURN;
}


static double _fsmb_lastVel = -1;

flight_state_t flight_state_motor_burn(system_data* data) {
	storage_write_to_flash(true);

	if (_fsmb_lastVel == -1) {
		_fsmb_lastVel = flightData.approxVelocity;
		return MOTOR_BURN;
	}
	if (_fsmb_lastVel < flightData.approxVelocity) {
		_fsmb_lastVel = flightData.approxVelocity;
		return MOTOR_BURN;
	}

	if (flightData.approxVelocity < 0) {
		flightData.apogeeReached = 1;
		return DESCENT;
	}

	flight_log("Detected motor burnout");
	flightData.activeControlsPermitted = 1;
	return ASCENT;
}

flight_state_t flight_state_ascent(system_data* data) {
	if (flightData.approxVelocity > 0) return ASCENT;
	flightData.apogeeReached = 1;

	char* message = smintf("Detected apogee (%f ft)", (flightData.currAlt - flightData.groundAlt));
	flight_log(message);
	free(message);

	fire_ejection_charge(DROGUE);

	return DESCENT;
}

flight_state_t flight_state_descent(system_data* data) {
	if (flightData.currAlt - flightData.groundAlt < MAIN_PARACHUTE_ALT_FT) {
		fire_ejection_charge(MAIN);
	}
	if (flightData.currAlt > flightData.groundAlt + 100) return DESCENT;
	if (abs(flightData.approxVelocity) > 10) return DESCENT;


	// LANDED

	storage_write_to_flash(false);
	storage_finish_flight(data);

	char* message = smintf("Landed at altitude %f", (float) (flightData.currAlt - flightData.groundAlt));
	flight_log(message);
	free(message);


	return LANDED;
}

flight_state_t flight_state_landed(system_data* data) {

	activate_beepx();
	return LANDED;
}

float interpolate_height_agl() {
	return (flightData.currAlt - flightData.groundAlt) + flightData.approxVelocity * (HAL_GetTick() - flightData.lastUpdateMs) / 1000;
}

void flight_log(char* message) {
	f_printf(f_flight_log, "[%d] %s\n", (int) HAL_GetTick(), message);
	f_sync(f_flight_log);

	log_message(message);
}

const flight_data_t flight_data() {
	return (const flight_data_t) flightData;
}

void fire_ejection_charge(ejection_type_t type) {

}

void activate_beepx() {

}
