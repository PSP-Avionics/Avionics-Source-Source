/*
 * active_controls.c
 *
 *  Created on: Jun 23, 2022
 *      Author: Alex Bowman
 *
 *  Code for managing when to deploy drag fins, and activating/managing the actual deployment mechanism
 */

#include "main.h"
#include "flight.h"
#include "active_controls.h"

static void active_controls_heartbeat(system_data* data);


void init_active_controls(system_data* data) {
	heartbeat_entry* ac_entry = calloc(1, sizeof(heartbeat_entry));
	ac_entry->function = active_controls_heartbeat;
	ac_entry->interval = ACTIVE_CONTROLS_INTERVAL;
	ac_entry->next = null;
	ac_entry->timeUntilNext = 2000;
	ac_entry->name = "active controls";

	register_heartbeat_func(ac_entry);

}

static void active_controls_heartbeat(system_data* data) {
	const flight_data_t flight = flight_data();
	if (!flight.activeControlsPermitted) {
		retract_airbrakes();
		return;
	}

	if (flight.apogeeReached) {
		retract_airbrakes();
		return;
	}

	// TODO : active controls stuff (assume it is permitted now)

	vec3f accel_vec;
	accel_acceleration(&accel_vec);

	float height = interpolate_height_agl();
	float accel = accel_vec.z; // z is up for the MPU. Assuming that the MPU starts pointed up
	float vel = flight_data().approxVelocity;

	float current_apogee = predict_apogee(vel, accel, height, 0);
	float apogee_if_deployed = predict_apogee(vel, accel, height, AIRBRAKE_DRAG_COEFF);

	if (apogee_if_deployed < TARGET_APOGEE) {
		retract_airbrakes();
	} else {
		deploy_airbrakes();
	}

}

static float predict_apogee(float vel, float accel, float height, float extraDragCoeff) {
	// x = x_0 + vt + 1/2at^2
	// v = v_0 + at
	// t = (v-v_0)/a = -v_0/a (v = 0 at apogee)

	float time = -vel/accel;
	return height + vel * time + 0.5f * accel * time * time; // TODO : use drag equation for acceleration (since it's not constant)
}

static void retract_airbrakes() {

}

static void deploy_airbrakes() {

}

