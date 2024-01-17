/*
 * rfm96.h
 *
 *  Created on: May 8, 2023
 *      Author: Alex Bowman
 */

#ifndef RFM96_H
#define RFM96_H

#include "stm32f4xx_hal.h"
#include "settings.h"

#include <stdint.h>

// Define SPI handle
extern SPI_HandleTypeDef RFM96_SPI_HANDLE;

// Define RFM96 pins
#define RFM96_NSS_Pin LoRa_CS_Pin
#define RFM96_NSS_GPIO_Port LoRa_CS_GPIO_Port
#define RFM96_RST_Pin GPIO_PIN_13
#define RFM96_RST_GPIO_Port GPIOB

// Function prototypes
void RF96_Init(float frequency, float bandwidth);
void RFM96_TransmitAFSK(uint8_t* data, uint16_t length);


/*
// Function prototypes
void rfm96_init(SPI_TypeDef *spi, GPIO_TypeDef *nss_port, uint16_t nss_pin, float freq_mhz, float bw_khz);
void rfm96_send_afsk_data(const uint8_t *data, uint32_t size);
*/

#endif /* RFM96_H */

