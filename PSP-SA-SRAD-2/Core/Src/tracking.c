/*
 * tracking.c
 *
 *  Created on: Nov 27, 2022
 *      Author: Alex Bowman
 */


#include "lib/gps.h"
#include "main.h"
#include "radio/telemetry.h"
#include "smintf.h"

extern GPS_t GPS;


float tracking_currentLat() {
	return (float) GPS.nmea_latitude;
}

float tracking_currentLong() {
	return (float) GPS.nmea_longitude;
}

static void tracking_heartbeat(system_data* data) {
	char* message = smintf("lat:%f,long:%f,alt_ft:%f",(double) GPS.nmea_latitude, (double) GPS.nmea_longitude, (double) GPS.altitude_ft); // TODO : APRS format it

	log_message(message);
//	telemetry_send(message);
//	telemetry_send("W9FTX");

	free(message);
}

void init_tracking(system_data* data) {
	GPS.nmea_latitude = 1.0f/0; // set default values to nan
	GPS.nmea_longitude = 1.0f/0;

	GPS_Init();

	heartbeat_entry* tracking_entry = calloc(1, sizeof(heartbeat_entry));
	tracking_entry->function = tracking_heartbeat;
	tracking_entry->interval = TRACKING_INTERVAL_MS;
	tracking_entry->next = null;
	tracking_entry->timeUntilNext = 0; // give 3ms so hopefully tasks dont overlap, plus 150 MS so that the BME280 device can take at least one sample
	tracking_entry->name = "tracking";

	register_heartbeat_func(tracking_entry);


}


