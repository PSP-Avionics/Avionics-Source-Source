/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/*
 *  ffconf.h includes main.h, but main.h must be loaded after ffconf, so that's what this weird ifdef stuff is about
 */
#include "ffconf.h"
#ifndef __MAIN_H_FFCONF
#define __MAIN_H_FFCONF
#define __MAIN_H
#define __BAD_MAIN_H1
#define __BAD_MAIN_H2
#endif

#ifdef __BAD_MAIN_H1
#undef __BAD_MAIN_H1
#else
#ifdef __BAD_MAIN_H2
#undef __BAD_MAIN_H2
#undef __MAIN_H
#endif
#endif


//#define DEBUG

/*
 * Again, look at message above if you're confused about the whacko list of ifdefs above
 *
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ff.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef void (heartbeat_func_t)(struct system_data_t* data);

typedef struct heartbeat_entry_t {
	heartbeat_func_t* function;
	uint16_t interval;
	int32_t timeUntilNext;
	char* name;
	struct heartbeat_entry_t* next;
} heartbeat_entry;

typedef struct system_data_t {
	heartbeat_entry* main_entry;
	FIL* file; // MAKE SURE to delete the #include main.h in ffconf.h if there is an error here
	FATFS* fs;
	bool fileOpen;
} system_data;


// utils

typedef struct vec3f_t {
	float x;
	float y;
	float z;
} vec3f;

float vec3f_len(vec3f vec);

//SPI_HandleTypeDef hspi2;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */



/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

uint32_t get_time_us();
void delay_us(uint64_t delay);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

//#define SD_DEBUG // enable if we want extra debug info in the output file

#define SD_SPI_HANDLE hspi2
#define SD_CS_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_9 // connected to D8, which is pin PA9

void led_blink(uint8_t);
void led_blink_delay(uint8_t, uint16_t);
void led_blink_error(uint8_t);
void register_heartbeat_func(heartbeat_entry* entry);
void log_message(const char* message);
void log_messagef(const char* message, ...);



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
