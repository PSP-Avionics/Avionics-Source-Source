/*
 * fs_storage.h
 *
 *  Created on: Sep 22, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_FS_STORAGE_H_
#define INC_FS_STORAGE_H_


#include <stdbool.h>
#include "settings.h"
#include "main.h"


void init_storage(system_data* data);

void storage_log(system_data* data, char* message);
void storage_finish_flight(system_data* data);
void storage_write_to_flash(bool newVal);

void check_ff_status(FRESULT result, char* desc);

uint16_t get_file_count(char* dir_loc);
void mkfile_and_dump(char* path, uint8_t* data, uint32_t len);


#endif /* INC_FS_STORAGE_H_ */
