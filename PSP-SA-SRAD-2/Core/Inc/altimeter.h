/*
 * altimeter.h
 *
 *  Created on: May 7, 2023
 *      Author: Alex Bowman
 */

#ifndef INC_ALTIMETER_H_
#define INC_ALTIMETER_H_

#include <stdlib.h>
#include <stdio.h>


#include "main.h"

#define ALT_SAMPLE_MAX_TRIES 5

#undef SAMPLE_HUMIDITY // define SAMPLE_HUMIDITY if using BME280, otherwise (if using BMP280), undefine this

static int8_t bmp280_i2c_reg_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
static int8_t bmp280_i2c_reg_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

void init_altimeter(system_data* data);
static void altimeter_heartbeat(system_data* data);

static void query_sensor();
double get_altitude_feet();
double get_last_pressure_data();
double get_last_temperature_data();
int32_t get_last_pressure_data_raw();
int32_t get_last_temperature_data_raw();


#include "lib/bmp280.h"


#endif /* INC_ALTIMETER_H_ */
