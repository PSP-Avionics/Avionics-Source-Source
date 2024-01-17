/*
 * altimeter.c
 *
 *  Created on: May 17, 2022
 *      Author: Alex Bowman
 *
 *  Code for detecting and reporting rocket altitude ASL (NOT AGL)
 */


#include <altimeter_old.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h" // for HAL_Delay
#include "lib/bme280.h"
#include "lib/bmp280.h"
#include "main.h"
#include "settings.h"

#ifdef SAMPLE_HUMIDITY
#define BME280_FLAGS (BME280_PRESS | BME280_TEMP | BME280_HUM)
#else
#define BME280_FLAGS (BME280_PRESS | BME280_TEMP)
#endif

static uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
void bmp280_delay_ms(uint32_t period, void* intf_ptr);
void bme280_delay_us(uint32_t period, void* intf_ptr);
static void init_bme280_dev(struct bme280_dev* dev_old, int8_t* rslt, uint32_t* expected_delay, bme280_read_fptr_t read, bme280_write_fptr_t write);
static void init_bmp280_dev();
static int8_t get_sensor_data(uint8_t flags, struct bme280_data* data, void* dev);
int8_t ioSpiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t ioSpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

int8_t ioI2CRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
int8_t ioI2CWrite(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);


static struct bme280_dev* dev_bme;
static struct bme280_data data, data_orig;
static int8_t last_result;
static void altimeter_heartbeat(system_data* sys_data);
static int8_t is_pressure_data_ok(struct bme280_data* data);

static struct bmp280_dev* dev;

extern SPI_HandleTypeDef BME280_SPI_HANDLE;
extern I2C_HandleTypeDef BMP280_I2C_HANDLE;

uint32_t init_rslts = 0;

void old_init_altimeter(system_data* data) {
	struct bme280_dev dev;
	int8_t* init_rslts_arr = &init_rslts;
	uint32_t expected_bme280_delay = 0;

	FIL* file = data->file;

#if THP_IS_BME280 == true
	init_bme280_dev(&dev, &init_rslts, &expected_bme280_delay, ioSpiRead, ioSpiWrite);
#else
	init_bmp280_dev();
#endif

	f_printf(&file, "BME280 Initialization results\ninit device,set_sensor_settings,set_sensor_mode\n%d,%d,%d\nMinimum Delay:,%d\n\n", init_rslts_arr[0], init_rslts_arr[1], init_rslts_arr[2], expected_bme280_delay);
	f_sync(&file);

}

static void init_bmp280_dev() {
	dev = (struct bmp280_dev*) calloc(1, sizeof(struct bmp280_dev));

	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET); // in I2C mode, the pin should remain active-high

	dev->delay_ms = bmp280_delay_ms;
	dev->intf = BMP280_I2C_INTF;
	dev->read = ioI2CRead;
	dev->write = ioI2CWrite;
//	dev->intf_ptr = NULL;

	int8_t rslt = bmp280_init(dev);

	struct bmp280_config conf;

    rslt = bmp280_get_config(&conf, dev);
//    conf.filter = BMP280_FILTER_COEFF_16;
//    conf.os_pres = BMP280_OS_4X;
//    conf.odr = BMP280_ODR_0_5_MS;

    conf.filter = BMP280_FILTER_COEFF_2;
    conf.os_pres = BMP280_OS_4X;
    conf.os_temp = BMP280_OS_4X;
    conf.odr = BMP280_ODR_1000_MS;

    rslt = bmp280_set_config(&conf, dev);

    rslt = bmp280_set_power_mode(BMP280_FORCED_MODE, dev);

    int8_t remaining_tries = ALT_SAMPLE_MAX_TRIES;

	 while (!is_pressure_data_ok(&data) && (remaining_tries--) > 0) // make sure we never sample pressure at 0
			last_result = get_sensor_data(0, &data, dev_bme);

}

static void init_bme280_dev(struct bme280_dev* dev_old, int8_t* rslt, uint32_t* expected_delay, bme280_read_fptr_t read, bme280_write_fptr_t write) {

	dev_bme = (struct bme280_dev*) calloc(1, sizeof(struct bme280_dev));

	// look at https://github.com/BoschSensortec/BME280_driver for help

	dev_bme->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev_bme->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev_bme->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev_bme->settings.filter = BME280_FILTER_COEFF_16;
	dev_bme->settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	dev_bme->intf_ptr = &dev_addr;
	dev_bme->intf = BME280_SPI_INTF;
	dev_bme->read = read;
	dev_bme->write = write;
	dev_bme->delay_us = bme280_delay_us;

	uint8_t settings_sel = BME280_OSR_PRESS_SEL
			| BME280_OSR_TEMP_SEL
#ifdef SAMPLE_HUMIDITY
			| BME280_OSR_HUM_SEL
#endif
			| BME280_FILTER_SEL
//			| BME280_STANDBY_SEL
	;

	rslt[0] = bme280_init(dev_bme);
	rslt[1] = bme280_set_sensor_settings(settings_sel, dev_bme);
	rslt[2] = bme280_set_sensor_mode(BME280_FORCED_MODE, dev_bme);

	*expected_delay = bme280_cal_meas_delay(&(dev_bme->settings));

	 heartbeat_entry* alt_entry = calloc(1, sizeof(heartbeat_entry));
	 alt_entry->function = altimeter_heartbeat;
	 alt_entry->interval = *expected_delay;
	 alt_entry->next = null;
	 alt_entry->timeUntilNext = *expected_delay;
	 alt_entry->name = "altimeter";

	 register_heartbeat_func(alt_entry);

	 int8_t remaining_tries = ALT_SAMPLE_MAX_TRIES;

	 while (!is_pressure_data_ok(&data) && (remaining_tries--) > 0) // make sure we never sample pressure at 0
	 		last_result = get_sensor_data(BME280_FLAGS, &data, dev_bme);


}

void old_query_pressure(struct bme280_data* data_out, int8_t* res) {
	if (res != null)
		*res = last_result;

	data_out->pressure = data.pressure;
	data_out->temperature = data.temperature;
#ifdef SAMPLE_HUMIDITY
	data_out->humidity = data.humidity;
#else
	data_out->humidity = 0;
#endif

}

static int8_t get_sensor_data(uint8_t flags, struct bme280_data* data, void* dev) {
#if THP_IS_BME280 == true
	return get_sensor_data(flags, data, (struct bme280_dev*) dev);
#else
	struct bmp280_uncomp_data ucomp_data;
	struct bmp280_dev* bmp = (struct bmp280_dev*) dev;
	bmp280_get_uncomp_data(&ucomp_data, bmp);
	// i know this is bad practice. on a time crunch right now
	double pres, temp;
	bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
	data->pressure = pres;
	data->temperature = temp;
	return 0;
#endif
}

static void altimeter_heartbeat(system_data* sys_data) {
	last_result = get_sensor_data((BME280_FLAGS), &data_orig, dev_bme);

#ifdef THP_IS_BME280 == true
	if (is_pressure_data_ok(&data_orig)) { // if false, still waiting for sensor to complete measurement
#else
	if (true) {
#endif

		data.pressure = data_orig.pressure;
		data.temperature = data_orig.temperature;
#ifdef SAMPLE_HUMIDITY
		data.humidity = data_orig.humidity;
#else
		data.humidity = 0;
#endif

		data_orig.pressure = 0; // make it look like it isnt working so when we sample next we will know

#if THP_IS_BME280 == true
		bme280_set_sensor_mode(BME280_FORCED_MODE, dev_bme);
#else
		bmp280_set_power_mode(BMP280_FORCED_MODE, dev);
#endif
	}
}


double old_get_altitude_meters(struct bme280_data* data, double* output) {

	double P_inHg = 0.000295300 * data->pressure;
	double P_mb = 33.8639 * P_inHg;

	// Using the formula from this website
	// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
	// P_sta is in mb, so P_sta = P_mb
//	double H_alt_m = (0.3048) * (1 - pow((P_mb / 1013.25), 0.190284)) * 145366.45;

	// new formula which includes temperature from https://keisan.casio.com/exec/system/1224585971#!
	double H_alt_m = (pow(101325.0 / data->pressure, 1.0/5.257) - 1) * (data->temperature + 273.15)/ 0.0065;

	if (output != null)
		*output = H_alt_m;

	return H_alt_m;
}

double old_get_altitude_feet(struct bme280_data* data, double* output) {
	double alt_m = get_altitude_meters(data, output);

	if (output != null)
		*output /= 0.3048;

	return alt_m / 0.3048;
}


void bmp280_delay_ms(uint32_t period, void* intf_ptr) {
	HAL_Delay(period);
}

void bme280_delay_us(uint32_t period, void* intf_ptr) {
	HAL_Delay(period/1000);
}

static int8_t is_pressure_data_ok(struct bme280_data* data) {
	return data->pressure != 0 && data->pressure != 30000 && ((int) data->pressure) != 64759;
}



int8_t ioSpiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

	// thp stands for temperature, humidity, and pressure
	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&BME280_SPI_HANDLE, &reg_addr, 1, 1000);
	HAL_StatusTypeDef status = HAL_SPI_Receive(&BME280_SPI_HANDLE, reg_data, len, 5000);

#ifdef SD_DEBUG
	if (len >= 4) {
		char* str = smintf("Send: %x,Rx: %x\n", (int) reg_addr, (int) *reg_data);
		f_printf(&file, str);
		free(str);
	} else {
		char* str = smintf("Send: %x,Rx: %x\n", (int) reg_addr, (int) reg_data[0]);
		f_printf(&file, str);
		free(str);
	}
#endif

	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET);

	if (status == HAL_OK)
		return 0;
	return -1;

}

int8_t ioSpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                                    void *intf_ptr) {

	// https://github.com/BoschSensortec/BME280_driver
    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_RESET);

	HAL_StatusTypeDef status = HAL_SPI_Transmit(&BME280_SPI_HANDLE, &reg_addr, 1, 1000);


	if (status != HAL_OK) {
		HAL_GPIO_WritePin(GPIOB, PIN_THP_CHIP_SELECT, GPIO_PIN_SET);
		return -1;
	}

	status = HAL_SPI_Transmit(&BME280_SPI_HANDLE, reg_data, (uint16_t) len, 5000);

	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET);

	if (status == HAL_OK)
			return 0;
		return -1;


}

// uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len
int8_t ioI2CWrite(uint8_t dev_id, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) {
	uint8_t* new_reg_data = malloc(len+1);
	new_reg_data[0] = reg_addr;
	for (int i = 0; i < len; i ++) {
		new_reg_data[i+1] = reg_data[i];
	}

	uint16_t slave_addr = 0b1110111 << (1 + 8);

	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET);
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&BMP280_I2C_HANDLE, slave_addr, new_reg_data, (uint16_t) len, 1000);

	free(new_reg_data);

	return 0;
}

int8_t ioI2CRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len) {
	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET);

	uint16_t slave_addr = 0b1110111 << (1 + 8);

	HAL_I2C_Master_Transmit(&BMP280_I2C_HANDLE, slave_addr, &reg_addr, (uint16_t) 1, 1000);

	HAL_I2C_Master_Receive(&BMP280_I2C_HANDLE, slave_addr, reg_data, (uint16_t) len, 1000);

//	HAL_GPIO_WritePin(GPIO_THP_CHIP_SELECT, PIN_THP_CHIP_SELECT, GPIO_PIN_SET); // pin should stay active-high for I2C

	return 0;

}


int8_t* old_altimeter_init_rslts_arr() {
	return &init_rslts;
}
