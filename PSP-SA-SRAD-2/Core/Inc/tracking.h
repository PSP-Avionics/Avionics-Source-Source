/*
 * tracking.h
 *
 *  Created on: Nov 27, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_TRACKING_H_
#define INC_TRACKING_H_

#include "lib/gps.h"

void init_tracking(system_data* data);

float tracking_currentLat();
float tracking_currentLong();

#endif /* INC_TRACKING_H_ */
