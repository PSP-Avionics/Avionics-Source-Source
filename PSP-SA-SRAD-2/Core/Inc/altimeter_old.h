/*
 * altimeter.h
 *
 *  Created on: May 17, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_ALTIMETER_OLD_H_
#define INC_ALTIMETER_OLD_H_


#include <stdlib.h>
#include <stdio.h>

#include "lib/bme280.h"
#include "main.h"

#define ALT_SAMPLE_MAX_TRIES 5

#undef SAMPLE_HUMIDITY // define SAMPLE_HUMIDITY if using BME280, otherwise (if using BMP280), undefine this

void old_init_altimeter(system_data* data);
static void altimeter_heartbeat(system_data* data);
void old_query_pressure(struct bme280_data* data, int8_t* result);
double old_get_altitude_meters(struct bme280_data* data, double* output);
double old_get_altitude_feet(struct bme280_data* data, double* output);

int8_t* altimeter_init_rslts_arr();

#endif /* INC_ALTIMETER_OLD_H_ */
