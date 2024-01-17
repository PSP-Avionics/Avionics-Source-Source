/*
 * fs_storage.c
 *
 *  Created on: Sep 22, 2022
 *      Author: Alex Bowman
 */

#include "fs_storage.h"
#include "lib/W25Q.h"

static FATFS fs;
static FIL file;
static FRESULT f_status;

#ifdef ENABLE_FLASH

static W25Q_t* flash_ptr;
static bool writeToFlash;
extern SPI_HandleTypeDef* W25Q_SPI;

static void init_flash (system_data* data) {
	log_message("initializing flash");

	flash_ptr = (W25Q_t*) malloc(sizeof(W25Q_t));
	wq_init(flash_ptr);
	wq_chip_erase(flash_ptr);

	log_message("flash init complete. hopefully it works!");
}

#endif

void init_storage(system_data* data) {
	data->file = &file;
	data->fs = &fs;

	// mount the file system

	log_message("mounting file system");
	log_message("mounting file system");
	log_message("mounting file system");

	HAL_Delay(500);

	log_message("mounting file system");

	f_status = f_mount(&fs, "/", 1);
	check_ff_status(f_status, "mount root");

	if (f_status != FR_OK) {
		data->fileOpen = false;
		log_messagef("error mounting file system (errno %d).", (int) f_status);
		return;
	}

	log_message("mounted file system");

	// try to open a file

	char* fileName = smintf("/alt/data%d.csv", (int) get_file_count("/alt"));
	f_status = f_open(&file, fileName, FA_CREATE_ALWAYS | FA_WRITE);
	data->fileOpen = f_status == FR_OK;
	check_ff_status(f_status, "create file"); // these checks might be kinda redundant, but oh well

	if (f_status != FR_OK) {
		log_messagef("could not create file %s (errno %d)", fileName, (int) f_status);
		return;
	}

	log_messagef("Logging data to file %s", fileName);

	char* header = FILE_HEADER;
	f_printf(&file, header);
	f_sync(&file);

	log_message("created file");


	free(fileName); // dont forget to free memory called with smintf

#ifdef ENABLE_FLASH
	init_flash(data);
#endif

}

void storage_log(system_data* data, char* message) {
	f_printf(data->file, message);
	f_printf(data->file, "\n");
	f_sync(data->file);

#ifdef ENABLE_FLASH
	if (writeToFlash)
		wq_write_bytes(flash_ptr, message, (int32_t) strlen(message));
#endif
}

void mkfile_and_dump(char* path, uint8_t* data, uint32_t len) {
	FILE file;
	f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
	UINT bw;
	while (bw < len)
		f_write(&file, data+bw, len-bw, &bw);
	f_sync(&file);
	f_close(&file);
}

// flight is finished, so we dont need to worry about vibrations effecting our SD card
// so we will copy from flash to SD card
void storage_finish_flight(system_data* data) {
#ifdef ENABLE_FLASH
	FILE file;
	f_open(&file, "/flash_dump.bl", FA_CREATE_ALWAYS | FA_WRITE);

	UINT bw;

	for (unsigned int i = 0; i <= flash_ptr->next_write_page+1; i ++) {
		wq_read_page(flash_ptr, i);
		f_write(&file, flash_ptr->read_buffer, 256, &bw);
	}

	f_sync(&file);
	f_close(&file);
#endif
}

uint16_t get_file_count(char* dir_loc) {
	char* nameSoWeEnterWhileLoop = "y";

	DIR dir;
	FILINFO fno = {
			.fname = nameSoWeEnterWhileLoop
	};
	f_mkdir(dir_loc);
	f_opendir(&dir, dir_loc);
	FRESULT result = 0;
	uint16_t i = 65535; // force cause an overflow, so when we run while loop for the first time, i = 0
	while (result == 0 && fno.fname[0] != 0) {

		i++;
		result = f_readdir(&dir, &fno);

	}
	return i;
}

void storage_write_to_flash(bool newVal) {
#ifdef ENABLE_FLASH
	writeToFlash = newVal;
#endif
}


// these functions down below are just for debugging purposes

void check_ff_status(FRESULT result, char* desc) {
	if (result == FR_OK) {
		led_blink_delay(3,50); // just something obvious
	} else {
		 uint8_t code = (uint8_t) result;

		 char* message = smintf("FATFS Error: %d %s\n", (int) code, desc);
		 log_message(message);
		 free(message);

#ifndef DEBUG
		 morse_code("sd error");

		  for (uint8_t i = 0; i < code; i ++) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_Delay(250);
		  }
#endif
	}

#ifndef DEBUG
	assert(result == FR_OK);
#endif
}
