/*
 * radiotest.c
 *
 *  Created on: May 8, 2023
 *      Author: Alex Bowman
 *
 *  What i told ChatGPT:

write code for me that will send an AFSK data over an RFM96 module in STM32. create for me the rfm96.c file if needed. Use SPI2 for communication with the RFM96, and assume it is already initialized properly. Assume the existance of a delay_us(uint32_t) function which will delay for a certain amount of microseconds. assume the NS pin is the CS pin, and the GPIO is already initialized for that pin


 */

//#include "rfm96.h"
#include "main.h"
#include "settings.h"
#include "stm32f4xx_hal.h"

#define RFM96_NSS_PIN LoRa_CS_Pin
#define RFM96_NSS_PORT LoRa_CS_GPIO_Port
#define RFM96_SPI hspi2
#define SPI_I2S_FLAG_BSY ((uint16_t)0x0080)

extern SPI_HandleTypeDef hspi2;


/*

// RFM registers
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_FRFMSB 0x06
#define REG_FRFMID 0x07
#define REG_FRFLSB 0x08
#define REG_PALEVEL 0x11
#define REG_LNA 0x18
#define REG_RXCONFIG 0x0D
#define REG_RSSICONFIG 0x0E
#define REG_RSSIVALUE 0x11
#define REG_IRQFLAGS1 0x27
#define REG_IRQFLAGS2 0x28
#define REG_DIOMAPPING1 0x25
#define REG_DIOMAPPING2 0x26

// RFM96 modes
#define MODE_SLEEP 0x00
#define MODE_STANDBY 0x01
#define MODE_FS 0x02
#define MODE_TX 0x03
#define MODE_RX 0x04

// RFM96 IRQ masks
#define IRQ1_MODE_READY 0x80
#define IRQ1_TX_READY 0x40
#define IRQ1_RX_READY 0x20
#define IRQ1_PAYLOAD_READY 0x04
#define IRQ1_CRC_OK 0x02
#define IRQ2_FIFO_NOT_EMPTY 0x40

// AFSK constants
#define MARK_FREQ 1200
#define SPACE_FREQ 2200
#define BAUD_RATE 1200
#define SAMPLE_RATE 8000

// Private functions
void rfm96_write_reg(uint8_t reg, uint8_t value);
uint8_t rfm96_read_reg(uint8_t reg);
void rfm96_set_mode(uint8_t mode);
void rfm96_wait_for_mode_ready();
void rfm96_wait_for_tx_ready();
void rfm96_wait_for_payload_ready();
void rfm96_wait_for_crc_ok();
void rfm96_wait_for_fifo_not_empty();
void rfm96_clear_irq_flags();

void rfm96_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin, float freq_mhz, float bw_khz) {
    // Configure NSS pin
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = nss_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(nss_port, &gpio_init);
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

    // Configure RFM96 module
    rfm96_write_reg(REG_OPMODE, MODE_SLEEP);
    uint32_t frf = (uint32_t)(freq_mhz * 1000000. / 61.03515625);
    rfm96_write_reg(REG_FRFMSB, (frf >> 16) & 0xFF);
    rfm96_write_reg(REG_FRFMID, (frf >> 8) & 0xFF);
    rfm96_write_reg(REG_FRFLSB, frf & 0xFF);
    uint8_t bw_exp = 0;
    while (bw_khz < 125000.0 && bw_exp < 7) {
        bw_khz *= 2.0;
        bw_exp++;
    }
    rfm96_write_reg(REG_RXCONFIG, 0x0E | (bw_exp << 3));
    rfm96_write_reg(REG_PALEVEL, 0x8F);
    rfm96_write_reg(REG_LNA, 0x88);
    rfm96_write_reg(REG_RSSICONFIG, 0x02);
    rfm96_write_reg(REG_RSSIVALUE, 0xE4);
    rfm96_write_reg(REG_DIOMAPPING1, 0x00);
    rfm96_write_reg(REG_DIOMAPPING2, 0x00);
}

void rfm96_send_afsk_data(const uint8_t *data, uint32_t size, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin) {
    // Set RFM96 to TX mode
    rfm96_set_mode(MODE_STANDBY);
    rfm96_write_reg(REG_DIOMAPPING1, 0x40);
    rfm96_write_reg(REG_DIOMAPPING2, 0x00);
    rfm96_clear_irq_flags();
    rfm96_set_mode(MODE_TX);

    // Send AFSK data
    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {
            uint8_t bit = (byte >> (7 - j)) & 1;
            uint16_t freq = bit ? MARK_FREQ : SPACE_FREQ;
            delay_us(1000000 / BAUD_RATE / 2);
            HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
            uint8_t tx_data[1] = {bit ? 0xFF : 0x00};
            HAL_SPI_Transmit(hspi, tx_data, 1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
            delay_us(1000000 / BAUD_RATE / 2);
        }
    }

    // Wait for TX to complete
    rfm96_wait_for_tx_ready();
    rfm96_set_mode(MODE_STANDBY);
}

void rfm96_write_reg(uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_RESET);
    uint8_t tx_data[2] = {reg | 0x80, value};
    HAL_SPI_Transmit(&hspi2, tx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_SET);
}

uint8_t rfm96_read_reg(uint8_t reg) {
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_RESET);
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};
    uint8_t rx_data[2];
    HAL_SPI_Transmit(&hspi2, tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, rx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_SET);
    return rx_data[1];
}

void rfm96_set_mode(uint8_t mode) {
    rfm96_write_reg(REG_OPMODE, (rfm96_read_reg(REG_OPMODE) & 0xE3) | (mode << 2));
}

void rfm96_wait_for_mode_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_MODE_READY) == 0);
}

void rfm96_wait_for_tx_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_TX_READY) == 0);
}

void rfm96_wait_for_payload_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_PAYLOAD_READY) == 0);
}

void rfm96_wait_for_crc_ok() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_CRC_OK) == 0);
}

void rfm96_wait_for_fifo_not_empty() {
    while ((rfm96_read_reg(REG_IRQFLAGS2) & IRQ2_FIFO_NOT_EMPTY) == 0);
}

void rfm96_clear_irq_flags() {
    rfm96_write_reg(REG_IRQFLAGS1, 0xFF);
    rfm96_write_reg(REG_IRQFLAGS2, 0xFF);
}
*/



// RFM registers
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_FRFMSB 0x06
#define REG_FRFMID 0x07
#define REG_FRFLSB 0x08
#define REG_PALEVEL 0x11
#define REG_LNA 0x18
#define REG_RXCONFIG 0x0D
#define REG_RSSICONFIG 0x0E
#define REG_RSSIVALUE 0x11
#define REG_IRQFLAGS1 0x27
#define REG_IRQFLAGS2 0x28
#define REG_DIOMAPPING1 0x25
#define REG_DIOMAPPING2 0x26

// RFM96 modes
#define MODE_SLEEP 0x00
#define MODE_STANDBY 0x01
#define MODE_FS 0x02
#define MODE_TX 0x03
#define MODE_RX 0x04

// RFM96 IRQ masks
#define IRQ1_MODE_READY 0x80
#define IRQ1_TX_READY 0x40
#define IRQ1_RX_READY 0x20
#define IRQ1_PAYLOAD_READY 0x04
#define IRQ1_CRC_OK 0x02
#define IRQ2_FIFO_NOT_EMPTY 0x40

// AFSK constants
#define MARK_FREQ 1200
#define SPACE_FREQ 2200
#define BAUD_RATE 1200
#define SAMPLE_RATE 8000

// Private functions
void rfm96_write_reg(uint8_t reg, uint8_t value);
uint8_t rfm96_read_reg(uint8_t reg);
void rfm96_set_mode(uint8_t mode);
void rfm96_wait_for_mode_ready();
void rfm96_wait_for_tx_ready();
void rfm96_wait_for_payload_ready();
void rfm96_wait_for_crc_ok();
void rfm96_wait_for_fifo_not_empty();
void rfm96_clear_irq_flags();

float rfm96_freq_mhz;
uint8_t rfm96_bw_exp;

uint32_t rfm96_frf;

void rfm96_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin, float freq_mhz, float bw_khz) {
    // Configure NSS pin
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = nss_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(nss_port, &gpio_init);
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

    // Configure RFM96 module
    rfm96_freq_mhz = freq_mhz;
    rfm96_write_reg(REG_OPMODE, MODE_SLEEP);
    rfm96_frf = (uint32_t)(freq_mhz * 1000000. / 61.03515625);
    rfm96_write_reg(REG_FRFMSB, (rfm96_frf >> 16) & 0xFF);
    rfm96_write_reg(REG_FRFMID, (rfm96_frf >> 8) & 0xFF);
    rfm96_write_reg(REG_FRFLSB, rfm96_frf & 0xFF);
    rfm96_bw_exp = 0;
    while (bw_khz < 125000.0 && rfm96_bw_exp < 7) {
        bw_khz *= 2.0;
        rfm96_bw_exp++;
    }
    rfm96_write_reg(REG_RXCONFIG, 0x0E | (rfm96_bw_exp << 3));
    rfm96_write_reg(REG_PALEVEL, 0x8F);
    rfm96_write_reg(REG_LNA, 0x88);
    rfm96_write_reg(REG_RSSICONFIG, 0x02);
    rfm96_write_reg(REG_RSSIVALUE, 0xE4);
    rfm96_write_reg(REG_DIOMAPPING1, 0x00);
    rfm96_write_reg(REG_DIOMAPPING2, 0x00);

    rfm96_set_mode(MODE_STANDBY);

}


void rfm96_send_afsk_data(const uint8_t *data, uint32_t size, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin) {
    // Set RFM96 to TX mode
    rfm96_set_mode(MODE_STANDBY);
//    rfm96_write_reg(REG_DIOMAPPING1, 0x40);
//    rfm96_write_reg(REG_DIOMAPPING2, 0x00);
//    rfm96_clear_irq_flags();
//    rfm96_set_mode(MODE_TX);

    // Send AFSK data
    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {
        	uint8_t bit = (byte >> (7 - j)) & 1;
            uint16_t freq = bit ? MARK_FREQ : SPACE_FREQ;
            uint32_t period_us = 1000000 / freq;
            uint32_t half_period_us = period_us / 2;
            for (uint32_t k = 0; k < (BAUD_RATE / 8); k++) {
                HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
                uint8_t tx_data[1] = {bit ? 0xFF : 0x00};
                HAL_SPI_Transmit(hspi, tx_data, 1, HAL_MAX_DELAY);
                HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
                delay_us(half_period_us);
            }
            delay_us(period_us);
        }
        rfm96_set_mode(MODE_TX);
        rfm96_write_reg(REG_FRFMSB, (rfm96_frf >> 16) & 0xFF);
        rfm96_write_reg(REG_FRFMID, (rfm96_frf >> 8) & 0xFF);
        rfm96_write_reg(REG_FRFLSB, rfm96_frf & 0xFF);
    }

    // Wait for TX to complete
//    rfm96_wait_for_tx_ready();
    rfm96_set_mode(MODE_STANDBY);
}

void rfm96_write_reg(uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_RESET);
    uint8_t tx_data[2] = {reg | 0x80, value};
    HAL_SPI_Transmit(&hspi2, tx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_SET);
}

uint8_t rfm96_read_reg(uint8_t reg) {
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_RESET);
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};
    uint8_t rx_data[2];
    HAL_SPI_Transmit(&hspi2, tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, rx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RFM96_NSS_PORT, RFM96_NSS_PIN, GPIO_PIN_SET);
    return rx_data[1];
}

void rfm96_set_mode(uint8_t mode) {
    rfm96_write_reg(REG_OPMODE, (rfm96_read_reg(REG_OPMODE) & 0xE3) | (mode << 2));
}

void rfm96_wait_for_mode_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_MODE_READY) == 0);
}

void rfm96_wait_for_tx_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_TX_READY) == 0);
}

void rfm96_wait_for_payload_ready() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_PAYLOAD_READY) == 0);
}

void rfm96_wait_for_crc_ok() {
    while ((rfm96_read_reg(REG_IRQFLAGS1) & IRQ1_CRC_OK) == 0);
}

void rfm96_wait_for_fifo_not_empty() {
    while ((rfm96_read_reg(REG_IRQFLAGS2) & IRQ2_FIFO_NOT_EMPTY) == 0);
}

void rfm96_clear_irq_flags() {
    rfm96_write_reg(REG_IRQFLAGS1, 0xFF);
    rfm96_write_reg(REG_IRQFLAGS2, 0xFF);
}

