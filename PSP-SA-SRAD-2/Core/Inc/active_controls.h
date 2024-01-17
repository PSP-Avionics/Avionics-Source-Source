/*
 * active_controls.h
 *
 *  Created on: Jun 23, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_ACTIVE_CONTROLS_H_
#define INC_ACTIVE_CONTROLS_H_

#define ACTIVE_CONTROLS_INTERVAL 10 // 100Hz (although note: sensors are less than that)
#define TARGET_APOGEE 10000
#define AIRBRAKE_DRAG_COEFF 0.6

void init_active_controls(system_data* data);

static void retract_airbrakes();
static void deploy_airbrakes();
static float predict_apogee(float vel, float accel, float height, float extraDragCoeff);

#endif /* INC_ACTIVE_CONTROLS_H_ */
