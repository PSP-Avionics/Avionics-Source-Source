/*==============================================================================================================
 * To use this library a buffer called LoRa_buff has been created
 * and we'll have to configure the parameters for LoRa in the RF95_Init()

1. include the declaration "extern uint8_t LoRa_buff[RH_RF95_FIFO_SIZE];" in the main.c.

2. Follow these steps for the sending/reception of data:

	-----------Send Mode-------------
	RF95_Init();
	strcpy(LoRa_buff, "Test to send"); //It is necessary to use the buffer LoRa_buff and the function strcpy to
									   //copy the data to the buffer, if not the program will send nothing.

	RF95_send(LoRa_buff);
	----------------------------------

	-----------Reception Mode-------------
	RF95_Init();
	RF95_setModeRx_Continuous();

	RF95_receive(LoRa_buff);
	----------------------------------

==============================================================================================================*/

/*
 * Code taken (and further modified) from https://github.com/MartiGeek17/LoRa-RFM95-STM32
 */




#ifndef _LORA_RF95_FSK_
#define _LORA_RF95_FSK_

#include "main.h"
#include "RF95_common.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#ifdef DEBUG
bool RF95FSK_send_try2_debug(uint8_t* data, uint16_t len);
#endif

void radio_heartbeat(system_data* data);
bool radio_enqueue(uint8_t* buffer, uint8_t len);
bool RF95FSK_send_try4(uint8_t* data, uint16_t len);

HAL_StatusTypeDef RF95_write(char reg, char wValue);
HAL_StatusTypeDef RF95_write_burst(char reg, uint8_t* data);
char RF95_read(char reg);
HAL_StatusTypeDef RF95_read_burst(char reg, char* buffer, int length);

//=====Init and configuration=====//
bool RF95FSK_Init(float freq_mhz, float bw_kHz, uint32_t bitrate);
void RF95FSK_Reset(void);
void RF95FSK_setTxPower(int8_t power, bool useRFO);

//=====Send and receive=====//
//bool RF95_receive(uint8_t* buf);
//bool RF95_receive_Timeout(uint8_t* buf, uint16_t timeout);
bool RF95FSK_send(uint8_t* data, uint16_t len);

//=====Control=====//
bool RF95FSK_available(void);
bool RF95FSK_available_Timeout(uint16_t timeout);
bool RF95FSK_available_Timeout(uint16_t timeout);
bool RF95FSK_waitPacketSent(void);
bool RF95FSK_waitCAD(void);

//=====Control Helper Functions=====//
static bool RF95FSK_setFrequency1i(uint32_t frf);
static bool RF95FSK_setFrequency1f(float centre);

//=====Modes Configuration=====//
void RF95FSK_setModeIdle(void);
bool RF95FSK_sleep(void);
void RF95FSK_setModeRx_Continuous(void);
void RF95FSK_setModeRx_Single(void);
void RF95FSK_setModeTx(void);
void RF95FSK_setModeTx_Continuous(void);
void RF95FSK_setModeCAD(void);
void RF95FSK_setOpMode(bool loraMode, bool fskMode);


//=====Check flags=====//
bool RF95FSK_Check_RxTimeout(void);
bool RF95FSK_Check_RxDone(void);
bool RF95FSK_Check_PayloadCRCError(void);
bool RF95FSK_Check_ValidHeader(void);
bool RF95FSK_Check_TxDone(void);
bool RF95FSK_Check_CADDone(void);
bool RF95FSK_Check_FhssChannelChange(void);
bool RF95FSK_Check_CADDetect(void);
void RF95FSK_Clear_IRQ(void);

bool RF95FSK_Check_ModemClear(void);
bool RF95FSK_Check_HeaderInfoValid(void);
bool RF95FSK_Check_RxOnGoing(void);
bool RF95FSK_Check_SignalSyncronized(void);
bool RF95FSK_Check_SignalDetect(void);


//=====CRC calculation=====//
uint16_t ComputeCRC(uint16_t crc, uint8_t data, uint16_t polynomial);
uint16_t RF95FSK_ComputeCRC(uint8_t *buffer, uint8_t bufferLength, uint8_t crcType);




//========================================================================
//=======================Communication protocol===========================
//========================================================================
/* The communication protocol uses three kinds of symbols
 * 		+ '?' -> to make petitions to a certain node
 * 		+ 'O' -> to say something has been done correctly or to continue
 * 		+ 'X' -> to say something has gone wrong or to stop
 * 	All of them are followed with the name of the node, which is only a number.
 */
#define Num_of_Nodes	2
#define Node_ID			1

void Clear_Buffer(uint8_t* buffer);
bool RF95FSK_Master_Receive_from_Node(uint16_t node);
bool RF95FSK_Master_Receive_from_All_Nodes(void);
bool RF95FSK_Slave_Send(void);

#endif


