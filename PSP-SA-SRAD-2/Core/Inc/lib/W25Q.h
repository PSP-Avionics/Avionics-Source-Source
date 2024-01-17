/*
 * W25Q.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Alexander Bowman
 */

#ifndef INC_LIB_W25Q_H_
#define INC_LIB_W25Q_H_

#include "stm32f4xx_hal.h"

#define WB_WRITE_ENABLE			0x06
#define WB_WRITE_DISABLE		0x04
#define WB_CHIP_ERASE			0xc7
#define WB_READ_STATUS_REG_1	0x05
#define WB_READ_DATA			0x03
#define WB_PAGE_PROGRAM			0x02
#define	WB_JEDEC_ID				0x9f

typedef struct {
	SPI_HandleTypeDef* spi;

	// this should be the gpio pin for the SPI output
	GPIO_TypeDef* gpio; // GPIOA, etc
	uint16_t gpio_pin; // GPIO_PIN_1

	uint8_t read_buffer[256];
	uint8_t first_byte, second_byte;
	uint8_t content[254];
	uint8_t content_size;
	unsigned int free_page_directory;

	uint16_t next_write_page;
	uint16_t next_write_offset;
} W25Q_t;

typedef uint16_t* word; // I assume this is correct

void wq_init(W25Q_t* ptr);
void wq_not_busy(W25Q_t* ptr);
void wq_get_jedec_id(W25Q_t* ptr);
void wq_chip_erase(W25Q_t* ptr);
void wq_read_page(W25Q_t* ptr, unsigned int page_number);
void wq_read_all_pages(W25Q_t* ptr);
void wq_write_byte(W25Q_t* ptr, word page, uint8_t offset, uint8_t databyte);
void wq_write_bytes(W25Q_t* ptr, uint8_t* buff, uint32_t len);


void wq_printBuffer(W25Q_t* ptr);
void wq_findEmptyPage(W25Q_t* ptr, uint8_t block_no);
void wq_printBufferHex(W25Q_t* ptr);

void wq_clearContent(W25Q_t* ptr);
void wq_extractContent(W25Q_t* ptr);

unsigned int wq_getFree_page_directory(W25Q_t* ptr);

uint8_t wq_getLastByte(W25Q_t* ptr);
uint8_t wq_getRead_buffer(W25Q_t* ptr, uint8_t b);
uint8_t wq_getFirst_byte(W25Q_t* ptr);
uint8_t wq_getSecond_byte(W25Q_t* ptr);
uint8_t wq_getContent(W25Q_t* ptr, uint8_t index);
uint8_t wq_getContent_size(W25Q_t* ptr);

void wq_setFree_page_directory(W25Q_t* ptr, unsigned int value);


// helper functions

static void _wq_get_jedec_id(W25Q_t* ptr, uint8_t* b1, uint8_t* b2, uint8_t* b3);
static void _wq_read_page(W25Q_t* ptr, word page_number, uint8_t* page_buffer);
static void _wq_read_page_internal(W25Q_t* ptr, word page_number, uint8_t* page_buffer);
static void _wq_write_page(W25Q_t* ptr, word page_number, uint8_t* page_buffer);

#endif /* INC_LIB_W25Q_H_ */
