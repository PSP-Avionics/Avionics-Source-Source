/*
 * prometheus.h
 *
 *  Created on: May 14, 2023
 *      Author: Alex Bowman
 */

#ifndef INC_PROMETHEUS_H_
#define INC_PROMETHEUS_H_

#include "main.h"

static void prometheus_heartbeat(system_data* data);
void init_prometheus(system_data* data);

#endif /* INC_PROMETHEUS_H_ */
