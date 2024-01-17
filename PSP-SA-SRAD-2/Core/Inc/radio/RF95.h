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




#ifndef _LORA_RF95_
#define _LORA_RF95_

#include "main.h"
#include "RF95_common.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


// Max number of bytes the LORA Rx/Tx FIFO can hold
#define RH_RF95_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_RF95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_RF95_MAX_MESSAGE_LEN
 #define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)
#endif


typedef enum
{
	Bw125Cr45Sf128 = 0,	   ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
	Bw500Cr45Sf128,	           ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	Bw31_25Cr48Sf512,	   ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	Bw125Cr48Sf4096,           ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
	IH_Bw125Cr45Sf128,  ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range + Implicit Header
} ModemConfigChoice;

typedef enum {
	RF95_BW_7_8kHz = 0,
	RF95_BW_10_4kHz = 1,
	RF95_BW_15_6kHz = 2,
	RF95_BW_20_8kHz = 3,
	RF95_BW_31_25kHz = 4,
	RF95_BW_41_7kHz = 5,
	RF95_BW_62_5kHz = 6,
	RF95_BW_125kHz = 7,
	RF95_BW_250kHz = 8,
	RF95_BW_500kHz = 9,

} ModemBandwidth;


HAL_StatusTypeDef RF95_write(char reg, char wValue);
HAL_StatusTypeDef RF95_write_burst(char reg, uint8_t* data);
char RF95_read(char reg);
HAL_StatusTypeDef RF95_read_burst(char reg, char* buffer, int length);

//=====Init and configuration=====//
bool RF95_Init(bool loraMode);
void RF95_Reset(void);
bool RF95_setModemConfig(ModemConfigChoice index);
void RF95_setModemConfig1bw(ModemBandwidth bw);
void RF95_setModemConfigFSK(uint16_t, uint16_t);
void RF95_setPreambleLength(uint16_t bytes);
void RF95_setTxPower(int8_t power, bool useRFO);
bool RF95_setFrequency(float centre);

//=====Send and receive=====//
bool RF95_receive(uint8_t* buf);
bool RF95_receive_Timeout(uint8_t* buf, uint16_t timeout);
bool RF95_send(uint8_t* data);
bool RF95_send_fskDirect(uint8_t* data, uint8_t len, uint16_t bitrate, float freqMidMHz, float offsetHz);

//=====Control=====//
bool RF95_available(void);
bool RF95_available_Timeout(uint16_t timeout);
bool RF95_available_Timeout(uint16_t timeout);
bool RF95_waitPacketSent(void);
bool RF95_waitCAD(void);

//=====Modes Configuration=====//
void RF95_setModeIdle(void);
bool RF95_sleep(void);
void RF95_setModeRx_Continuous(void);
void RF95_setModeRx_Single(void);
void RF95_setModeTx(void);
void RF95_setModeTx_Continuous(void);
void RF95_setModeCAD(void);
void RF95_setOpMode(bool loraMode, bool fskMode);


//=====Check flags=====//
bool RF95_Check_RxTimeout(void);
bool RF95_Check_RxDone(void);
bool RF95_Check_PayloadCRCError(void);
bool RF95_Check_ValidHeader(void);
bool RF95_Check_TxDone(void);
bool RF95_Check_CADDone(void);
bool RF95_Check_FhssChannelChange(void);
bool RF95_Check_CADDetect(void);
void RF95_Clear_IRQ(void);

bool RF95_Check_ModemClear(void);
bool RF95_Check_HeaderInfoValid(void);
bool RF95_Check_RxOnGoing(void);
bool RF95_Check_SignalSyncronized(void);
bool RF95_Check_SignalDetect(void);


//=====CRC calculation=====//
uint16_t ComputeCRC(uint16_t crc, uint8_t data, uint16_t polynomial);
uint16_t RF95_ComputeCRC(uint8_t *buffer, uint8_t bufferLength, uint8_t crcType);




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
bool RF95_Master_Receive_from_Node(uint16_t node);
bool RF95_Master_Receive_from_All_Nodes(void);
bool RF95_Slave_Send(void);

#endif


