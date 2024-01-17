/*
 * settings.h
 *
 * Moving all the settings and defines, etc into one location so it's easier to control
 *
 *  Created on: Nov 5, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#define null NULL

#define PIN_THP_SERIAL_IN D2
#define PIN_THP_SERIAL_OUT D3
#define	GPIO_THP_CHIP_SELECT GPIOE
#define PIN_THP_CHIP_SELECT GPIO_PIN_1
#define PIN_THP_SERIAL_CK D13

#define THP_IS_BME280 false // true if BME280, false if BMP280

#define BME280_SPI_HANDLE hspi1
#define BMP280_I2C_HANDLE hi2c2

#define LoRa_CS_GPIO_Port GPIOD
#define LoRa_CS_Pin GPIO_PIN_0
#define LoRa_RESET_GPIO_Port GPIOD
#define LoRa_RESET_Pin GPIO_PIN_1
#define TX_LED_GPIO_Port GPIOA
#define TX_LED_Pin GPIO_PIN_1
#define RX_LED_GPIO_Port GPIOA
#define RX_LED_Pin GPIO_PIN_1
#define LoRa_SPI hspi2

#define RADIO_FSK_MODE true
#define FREQUENCY 436.000f // freq in MHz
#define BANDWIDTH 12.0000f // bandwidth in kHz
//#define BANDWIDTH 1200.0000f // bandwidth in kHz
#define BITRATE 8 // bits per second
#undef TEST_RADIO // change from undef to define if you want to test the radio

#define MARK_FREQ 1200
#define SPACE_FREQ 2200
#define BAUD_RATE 1200

#undef ENABLE_FLASH // turn #undef into #define if you want to enable flash
#define W25Q_SPI hspi3
#define W25Q_GPIO_Pin 4
#define W25Q_GPIO_Port GPIOB

#define GPS_UART huart2

#define ACCELEROMETER_I2C_HANDLE hi2c1

#define MAIN_PARACHUTE_ALT_FT 1000 // main parachute deployment altitude in feet

#define TRACKING_INTERVAL_MS 5000

#define FILE_HEADER "Measured Values,,,,,,,Calculated Values\n\rTime (ms),Pressure (pascal),Temperature (°C),Humidity (%),,,Altitude (m),Altitude (ft),Appx. Velocity (ft/s),Flight State,GPS lat,GPS long,Rot x,Rot y,Rot z\n"


// some old/legacy settings (might be useless)
#define USB_ONLY // if the usb isn't working, undefine this and the led can be used for debugging purposes


#endif /* INC_SETTINGS_H_ */
