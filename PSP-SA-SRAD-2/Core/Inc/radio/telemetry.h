/*
 * telemetry.h
 *
 *  Created on: Aug 28, 2022
 *      Author: Alex Bowman
 */

#ifndef INC_RADIO_TELEMETRY_H_
#define INC_RADIO_TELEMETRY_H_

void init_telemetry(system_data* data);
void telemetry_send(const char* data);
void telemetry_RxCplt(SPI_HandleTypeDef* spi);
void telemetry_heartbeat(system_data* data);

#endif /* INC_RADIO_TELEMETRY_H_ */
