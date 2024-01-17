/*
 * prometheus.c
 *
 *  Created on: May 14, 2023
 *      Author: Alex Bowman
 *
 * Code targeted for the prometheus launch/SPAC 2023
 * Can be copied for future launches depending upon necessity
 */

#include "prometheus.h"

#define BUFFER_SIZE 580700// 58kB
#define SAMPLE_COUNT BUFFER_SIZE/(4*2)
#define SAMPLE_RATE 10

#define SAMPLE_FLUSH_RATE_MINS 45 // a 10 min launch, less than 25% chance of it a flush being during a launch

static uint8_t* buffer;
static uint32_t current_sample_count;

static void dump_and_reset_buffer(uint8_t isOkToKeepHalf) {
	char* path = smintf("/sensorbin/data%d.csv", (int) get_file_count("/sensorbin"));
	mkfile_and_dump(path, buffer, current_sample_count*4*2);
	free(path);

	if (isOkToKeepHalf) {
		if (current_sample_count < SAMPLE_COUNT/2); // we can store an extra half, so keep as much data as possible
		else {
			if (SAMPLE_FLUSH_RATE_MINS * 60 * SAMPLE_RATE < BUFFER_SIZE/8/2) {
				// delete some old data, but make a backup of the newish/just barely old data as well just in case
				uint32_t second_start_idx = current_sample_count - SAMPLE_FLUSH_RATE_MINS * 60 * SAMPLE_RATE;
				uint32_t* buffer_int = (uint32_t*) buffer;
				for (uint32_t i = 0; i < SAMPLE_FLUSH_RATE_MINS * 60 * SAMPLE_RATE * 2; i ++) {
					buffer_int[i] = buffer_int[second_start_idx+i];
				}
				current_sample_count = SAMPLE_FLUSH_RATE_MINS * 60 * SAMPLE_RATE;
			}
			current_sample_count = 0;
		}
	} else {
		current_sample_count = 0;
	}
}

static void prometheus_heartbeat(system_data* data) {
	int32_t* newBuffer = (int32_t*) buffer; // i'm too lazy to make a union and this works so i dont care
	newBuffer[current_sample_count*2] = get_last_pressure_data_raw(); // I dont know if this is Big Endian or LE but I will find out later
	newBuffer[current_sample_count*2+1] = get_last_temperature_data_raw();

	current_sample_count++;

	if (current_sample_count >= 60 * SAMPLE_RATE * SAMPLE_FLUSH_RATE_MINS) { // we are very close to ground, so assume little to no vibration from an actual launch
		dump_and_reset_buffer(1);
	}

	if (current_sample_count >= SAMPLE_COUNT) {
		dump_and_reset_buffer(0);
	}

}

void init_prometheus(system_data* data) {
	buffer = malloc(BUFFER_SIZE);
	log_messagef("created buffer for prometheus launch data successfully");
	log_messagef("buffer can hold %ds (%d min) of data before needing to dump", (int) (SAMPLE_COUNT/SAMPLE_RATE), (int) (SAMPLE_COUNT/SAMPLE_RATE/60));

	heartbeat_entry* prometheus_entry = calloc(1, sizeof(heartbeat_entry));
	prometheus_entry->function = prometheus_heartbeat;
	prometheus_entry->interval = 1000/SAMPLE_RATE;
	prometheus_entry->next = NULL;
	prometheus_entry->timeUntilNext = 5017; // 5 seconds because rocket will take a while before launch, and 17 since its a prime number and we want to space them out so theres no overlap really
	prometheus_entry->name = "prometheus";

	register_heartbeat_func(prometheus_entry);
}
