/*
 * W25Q.c
 *
 *	Using this as a reference: https://github.com/irsyadtc/Arduino_W25Q_flash_memory_library/blob/master/W25Q-1.0/W25Q.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: Alexander Bowman
 */

#include <math.h>
#include "lib/W25Q.h"
#include "main.h"
#include "math.h"
#include "settings.h"

extern SPI_HandleTypeDef W25Q_SPI;

#define __wq_write_low()
#define __wq_write_high()
#define __wq_begin_write()
#define __wq_spi_write(data)
#define __wq_spi_write_1p(data)
#define __wq_spi_read(byte_ptr)

//
//#define __wq_write_low() HAL_GPIO_WritePin(ptr->gpio, ptr->gpio_pin, GPIO_PIN_RESET)
//#define __wq_write_high() HAL_GPIO_WritePin(ptr->gpio, ptr->gpio_pin, GPIO_PIN_SET)
//#define __wq_begin_write() __wq_write_high(); delay_us(50); __wq_write_low()
//#define __wq_spi_write(data) {char dataVar = data; HAL_SPI_Transmit(ptr->spi, &(dataVar), sizeof(dataVar), 1000);}
//#define __wq_spi_write_1p(data) HAL_SPI_Transmit(ptr->spi, data, sizeof(data), 1000);
//#define __wq_spi_read(byte_ptr) HAL_SPI_Receive(ptr->spi, byte_ptr, sizeof(byte_ptr), 1000);
#define min(val1, val2) val1<val2?val1:val2

void wq_init(W25Q_t* ptr) {
	// assume spi has already been initialized in main
	// make sure spi is MSB first
	ptr->spi = &W25Q_SPI;
	ptr->gpio = W25Q_GPIO_Port;
	ptr->gpio_pin = W25Q_GPIO_Pin;
	ptr->free_page_directory = 0;
}

void wq_not_busy(W25Q_t* ptr) {
	__wq_begin_write();
	__wq_spi_write(WB_READ_STATUS_REG_1);
	__wq_write_high();
}

void wq_get_jedec_id(W25Q_t* ptr) {
	uint8_t b1, b2, b3;
	_get_jedec_id(ptr, &b1, &b2, &b3);
	// b1: manufacturer id
	// b2: memory type
	// b3: capacity
}

void wq_chip_erase(W25Q_t* ptr) {
	__wq_begin_write();
	__wq_spi_write(WB_WRITE_ENABLE);
	__wq_write_high();
	__wq_write_low();
	__wq_spi_write(WB_CHIP_ERASE);
	__wq_write_high();

	wq_not_busy(ptr);
}

void wq_read_page(W25Q_t* ptr, unsigned int page_number) {
	_wq_read_page(ptr, page_number, ptr->read_buffer);
}

void wq_read_all_pages(W25Q_t* ptr) {
	uint8_t page_buffer[256];

	for (int i = 0; i < 4096; i ++) {
		_wq_read_page(ptr, i, page_buffer);
		// do something here
	}
}

void wq_write_byte(W25Q_t* ptr, word page, uint8_t offset, uint8_t databyte) {
	uint8_t page_data[256];
	_wq_read_page_internal(ptr, page, page_data);
	page_data[offset] = databyte;
	_wq_write_page(ptr, page, page_data);
}

void wq_write_bytes(W25Q_t* ptr, uint8_t* buff, uint32_t len) {
	word page = &(ptr->next_write_page);

	uint32_t offset = ptr->next_write_offset;

	uint32_t initialPos = offset;
	uint32_t endPos = min(255-offset, len);
	uint8_t page_data[256];
	uint32_t idxInBuff = 0;
	while (len > 0) {
		_wq_read_page_internal(ptr, page, page_data);
		for (uint32_t i = offset; i < 256; i ++) {
			page_data[i] = buff[idxInBuff];
			idxInBuff++;
		}
		_wq_write_page(ptr, page, page_data);
		len -= (255-offset);
		offset = 0;
		*page++;
	}

	ptr->next_write_page = ptr->next_write_page + (len)/256+(ptr->next_write_offset+len%256)/256;
	ptr->next_write_offset = (ptr->next_write_offset+len%256)%256;
}

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

static void _wq_get_jedec_id(W25Q_t* ptr, uint8_t* b1, uint8_t* b2, uint8_t* b3) {
	__wq_begin_write();
	__wq_spi_write(WB_JEDEC_ID);
	__wq_spi_read(b1);
	__wq_spi_read(b2);
	__wq_spi_read(b3);
	__wq_write_high();
	wq_not_busy(ptr);

}

static void _wq_read_page(W25Q_t* ptr, word page_number, uint8_t* page_buffer) {
	__wq_begin_write();
	__wq_spi_write(WB_READ_DATA);
	__wq_spi_write_1p((uint8_t*) page_number);
	__wq_spi_write((uint8_t) 0);

	__wq_spi_read(page_buffer);

	__wq_write_high();
	wq_not_busy(ptr);
}

static void _wq_read_page_internal(W25Q_t* ptr, word page_number, uint8_t* page_buffer) {
	_wq_read_page(ptr, page_number, page_buffer);
}

static void _wq_write_page(W25Q_t* ptr, word page_number, uint8_t* page_buffer) {
	__wq_begin_write();
	__wq_spi_write(WB_WRITE_ENABLE);
	__wq_begin_write();
	__wq_spi_write(WB_PAGE_PROGRAM);
	__wq_spi_write_1p((uint8_t*) page_number);
	__wq_spi_write((uint8_t) 0);
	__wq_spi_write_1p(page_buffer);
	__wq_write_high();

	wq_not_busy(ptr);
}
