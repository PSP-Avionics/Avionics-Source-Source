/*
 * altimeter.c
 *
 *  Created on: May 7, 2023
 *      Author: Alex Bowman
 *
 *  Altimeter code for the BMP280 device (not BME280, so no humidity)
 */

#include <math.h>
#include "stm32f4xx_hal.h"
#include "altimeter.h"
#include "main.h"
#include "lib/bmp280.h"

#define BMP280_I2C_ADDRESS 0x76

extern I2C_HandleTypeDef* hi2c2;
extern SPI_HandleTypeDef* hspi1;

//
//static uint8_t BMP280_I2C_Read(uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//    if (HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//    if (HAL_I2C_Master_Receive(&hi2c2, BMP280_ADDRESS, data, len, HAL_MAX_DELAY) != HAL_OK)
//    {
//        return HAL_ERROR;
//    }
//    return HAL_OK;
//}
//
//static uint8_t BMP280_I2C_Write(uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//    uint8_t *buf = (uint8_t *)malloc(len + 1);
//    buf[0] = reg_addr;
//    memcpy(buf + 1, data, len);
//
//    if (HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDRESS, buf, len + 1, HAL_MAX_DELAY) != HAL_OK)
//    {
//        free(buf);
//        return HAL_ERROR;
//    }
//
//    free(buf);
//    return HAL_OK;
//}

#define BMP280_CS_GPIO_Port GPIOE
#define BMP280_CS_GPIO_Pin GPIO_PIN_0

static int8_t BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Select the BMP280 sensor
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_GPIO_Pin, GPIO_PIN_RESET);  // Set appropriate GPIO pin to low for chip select

    // Transmit the register address with the MSB cleared to indicate write operation
    uint8_t txData = reg_addr & ~0x80;
    if (HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Error in SPI transmission
    	Error_Handler();
        return BMP280_E_COMM_FAIL;
    }

    // Transmit the data
    if (HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        // Error in SPI transmission
    	Error_Handler();
        return BMP280_E_COMM_FAIL;
    }

    // Deselect the BMP280 sensor
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_GPIO_Pin, GPIO_PIN_SET);  // Set appropriate GPIO pin to high for chip deselect

    return BMP280_OK;
}

// Function to read data from the BMP280 sensor
static int8_t BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Select the BMP280 sensor
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_GPIO_Pin, GPIO_PIN_RESET);  // Set appropriate GPIO pin to low for chip select

    // Transmit the register address with the MSB set to indicate read operation
    uint8_t txData = reg_addr | 0x80;
    if (HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Error in SPI transmission
    	Error_Handler();
        return BMP280_E_COMM_FAIL;
    }

    // Receive the data
    if (HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        // Error in SPI reception
    	Error_Handler();
        return BMP280_E_COMM_FAIL;
    }

    // Deselect the BMP280 sensor
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_GPIO_Pin, GPIO_PIN_SET);  // Set appropriate GPIO pin to high for chip deselect

    return BMP280_OK;
}

static void i2c_check() {
	// Check I2C peripheral status
	HAL_I2C_StateTypeDef i2cState = HAL_I2C_GetState(&hi2c2);  // Replace hi2c1 with your I2C handle variable

	// Check the state and handle accordingly
	switch (i2cState) {
	    case HAL_I2C_STATE_RESET:
	        // I2C peripheral is in reset state
	    	log_messagef("i2c2 reset");
	        break;

	    case HAL_I2C_STATE_READY:
	        // I2C peripheral is ready and idle
	    	log_messagef("i2c2 ready");
	        break;

	    case HAL_I2C_STATE_BUSY:
	        // I2C peripheral is busy with a previous operation
	    	log_messagef("i2c2 busy");
	        break;

	    case HAL_I2C_STATE_BUSY_TX:
	        // I2C peripheral is busy transmitting data
	    	log_messagef("i2c2 tx");
	        break;

	    case HAL_I2C_STATE_BUSY_RX:
	        // I2C peripheral is busy receiving data
	    	log_messagef("i2c2 rx");
	        break;

	    case HAL_I2C_STATE_ERROR:
	        // I2C peripheral error occurred
	    	log_messagef("i2c2 err");
	        break;

	    case HAL_I2C_STATE_TIMEOUT:
	    	log_messagef("i2c2 state check timeout");
	        // I2C peripheral operation timeout
	        break;

	    default:
	        break;
	}
}

static uint32_t read_pressure(void)
{
    uint8_t data[3];
    uint32_t pressure;

    // set mode to forced, oversampling to 4x, and enable pressure measurement
    uint8_t ctrl_meas_reg = 0x27;
    BMP280_I2C_Write(0xF4, &ctrl_meas_reg, 1);

    // wait for measurement to complete (takes approx. 4.5ms)
    HAL_Delay(5);

    // read pressure data
    BMP280_I2C_Read(0xF7, data, 3);

    // combine data into 20-bit value
    pressure = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);

    // convert pressure to mb (1 mb = 100 Pa)
    pressure /= 100;

    return pressure;
}

struct bmp280_dev bmp;
static double alt_feet;


void init_altimeter(system_data* data) {

//
//    bmp.dev_id = BMP280_I2C_ADDR_PRIM;
//    bmp.intf = BMP280_I2C_INTF;
//    bmp.read = bmp280_i2c_reg_read;
//    bmp.write = bmp280_i2c_reg_write;
//    bmp.delay_ms = HAL_Delay;
//    int8_t rslt = bmp280_init(&bmp);
//    if (rslt != BMP280_OK) {
//    	log_messagef("BMP280 no ok big sad");
//        Error_Handler();
//    }


	bmp.dev_id = 0;
	bmp.intf = BMP280_SPI_INTF;
	bmp.read = BMP280_SPI_Read;
	bmp.write = BMP280_SPI_Write;
	bmp.delay_ms = HAL_Delay;
	int8_t rslt = bmp280_init(&bmp);
	if (rslt != BMP280_OK) {
		log_messagef("BMP280 no ok big sad");
		Error_Handler();
	}

	struct bmp280_config config = {
		    .os_temp = BMP280_OS_2X,
		    .os_pres = BMP280_OS_16X,
		    .odr = BMP280_ODR_0_5_MS,
		    .filter = BMP280_FILTER_COEFF_16,
		    .spi3w_en = BMP280_SPI3_WIRE_DISABLE
	};

	bmp280_set_config(&config, &bmp);
	bmp280_set_power_mode(BMP280_FORCED_MODE, &bmp);


    alt_feet = -1.0;

	heartbeat_entry* alt_entry = calloc(1, sizeof(heartbeat_entry));
	alt_entry->function = altimeter_heartbeat;
	alt_entry->interval = 100; // 100 ms for now
	alt_entry->next = NULL;
	alt_entry->timeUntilNext = alt_entry->interval;
	alt_entry->name = "altimeter-v2";

	register_heartbeat_func(alt_entry);
}

static void altimeter_heartbeat(system_data* data) {
	query_sensor(); // for now, just so it prints and I know its working
}

static int32_t last_raw_pressure, last_raw_temperature;
static double last_pressure, last_temperature;

static void query_sensor() {
	struct bmp280_uncomp_data ucomp_data;
	bmp280_get_uncomp_data(&ucomp_data, &bmp);

	bmp280_set_power_mode(BMP280_FORCED_MODE, &bmp);

	bmp280_get_comp_temp_32bit(&last_raw_temperature, ucomp_data.uncomp_temp, &bmp);
	bmp280_get_comp_pres_32bit(&last_raw_pressure, ucomp_data.uncomp_press, &bmp);

	bmp280_get_comp_pres_double(&last_pressure, ucomp_data.uncomp_press, &bmp);
	bmp280_get_comp_temp_double(&last_temperature, ucomp_data.uncomp_press, &bmp);

//    log_messagef("pressure: %f temp: %f", (double) last_pressure, (double) last_temperature);

}

double get_altitude_feet() {

    float press =  (float) last_pressure;
    float temp = (float) last_temperature;

//    log_messagef("pressure: %f temperature: %f", (double) press, (double) temp);

//    double P_inHg = 0.000295300 * press;
//	double P_mb = 33.8639 * P_inHg;
    double P_mb = press; // pressure is in hPa, but this I think is equivalent to 1hPa = 1mb

	// Using the formula from this website
	// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
	// P_sta is in mb, so P_sta = P_mb
//	double H_alt_m = (0.3048) * (1 - pow((P_mb / 1013.25), 0.190284)) * 145366.45;

	// new formula which includes temperature from https://keisan.casio.com/exec/system/1224585971#!
	double H_alt_m = (pow(101325.0 / press, 1.0/5.257) - 1) * (temp + 273.15)/ 0.0065;

//    log_messagef("pressure: %f temp: %f H_alt_m: %f", (double) last_pressure, (double) last_temperature, (double) H_alt_m);

	return H_alt_m / 0.3048;
}

double get_last_pressure_data() {
	return last_pressure;
}

double get_last_temperature_data() {
	return last_temperature;
}

int32_t get_last_pressure_data_raw() {
	return last_raw_pressure;
}

int32_t get_last_temperature_data_raw() {
	return last_raw_temperature;
}

static int8_t bmp280_i2c_reg_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    i2c_check();
	HAL_I2C_StateTypeDef i2cStatePre = HAL_I2C_GetState(&hi2c2);  // Replace hi2c1 with your I2C handle variable
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c2, BMP280_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 3000);
	HAL_I2C_StateTypeDef i2cState = HAL_I2C_GetState(&hi2c2);  // Replace hi2c1 with your I2C handle variable
	if (status == HAL_OK) {
        return BMP280_OK;
    } else {
        Error_Handler();
//    	log_messagef("i2c read failed");
        return BMP280_E_COMM_FAIL;
    }
}

static int8_t bmp280_i2c_reg_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c2, BMP280_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
	if (status == HAL_OK) {
		return BMP280_OK;
	} else {
        Error_Handler();

		return BMP280_E_COMM_FAIL;
	}
}


