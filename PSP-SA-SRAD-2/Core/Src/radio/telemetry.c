/*
 * telemetry.c
 *
 *  Created on: Aug 28, 2022
 *      Author: Alex Bowman
 */

#include "main.h"
#include "radio/telemetry.h"
#include "settings.h"
#ifdef RADIO_FSK_MODE
#include "radio/RF95_FSK.h"
#else
#include "radio/RF95.h" // RF95 should have very same protocol to RF96, so I'm gonna try it out for now
#endif

#include "radio/rfm96.h"


extern SPI_HandleTypeDef hspi2;
extern uint8_t LoRa_buff[];

uint8_t RX_Buffer[255] = {0};

static bool initOk;

extern uint16_t fdev0_override;
extern uint16_t bitrate0_override;

void reset_telemetry() {

#ifdef RADIO_FSK_MODE
	bool freqOk;
	RF95FSK_Reset();
	initOk = freqOk = RF95FSK_Init(FREQUENCY,BANDWIDTH,BITRATE);
	RF95FSK_setTxPower(14, false); // this function returns void, not bool
#else
	bool lora = false;
				  RF95_Reset();
		 initOk = RF95_Init(lora);
				  RF95_setOpMode(lora, true);
//	bool modmOk = RF95_setModemConfig(Bw31_25Cr48Sf512);
//	     	 	  RF95_setModemConfig1bw(RF95_BW_15_6kHz); // keep it as narrow as possible right now for my ham radio (20kHz BW on wide mode)
				  RF95_setModemConfigFSK(BITRATE, 15000); // keep it as narrow as possible right now for my ham radio (20kHz BW on wide mode)
	bool freqOk = RF95_setFrequency(FREQUENCY);
				  RF95_setTxPower(14, false); // this function returns void, not bool
#endif

	log_messagef("Telemetry status: %d %d (1 is good)", (int) initOk, (int) freqOk);

	HAL_SPI_Receive_IT(&hspi2, RX_Buffer, sizeof(RX_Buffer));

#ifdef TEST_RADIO
	telemetry_send("W9FTX");

	log_messagef("Telemetry initOk? %d", (int) initOk);

//	uint8_t data1[10] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xF0, 0xFF};
//	RF95FSK_send_try2_debug(data1, sizeof(data1));

	while(true) {
		HAL_Delay(2000);
//		telemetry_send("W9FTX abcdefghijklmnopqrstuvwxyz123456789 W9FTX abcdefghijklmnopqrstuvwxyz123456789");
		uint8_t data[4] = {0x00, 0xFF, 0b10101010, 0b11000011};
		for (uint8_t i = 0; i < sizeof(data); i ++) {
			RF95FSK_send_try4(data+i, 1);
			HAL_Delay(250);
		}
		HAL_Delay(250);
		telemetry_send("W9FTX");
//		for (int i = 0 ; i < sizeof(data) ; i ++) {
//			data[i] = i;
//		}
//		for (int i = 32; i <= 0xFFFF; i *= 2) {
//			for (int j = 32; j <= 0xFFFF; j *= 2) {
////				fdev0_override = i;
////				bitrate0_override = j;
//				for (uint8_t i = 0; i < sizeof(data); i ++) {
//					RF95FSK_send_try4(data+i, 1);
//					HAL_Delay(250);
//				}
////				RF95FSK_send_try4(data, sizeof(data));
//				HAL_Delay(1000);
//			}
//		}
	}
#endif

}


void init_telemetry(system_data* data) {
	reset_telemetry();

////  code below is chat-gpt generated. it sucks lol
//
//	RF95FSK_Init(FREQUENCY, BANDWIDTH, 1200);
//	RF95FSK_setModemConfigFSK(64, 100);
//	rfm96_init(&hspi2, LoRa_CS_GPIO_Port, LoRa_CS_Pin, FREQUENCY, BANDWIDTH);
////	RFM96_Init(FREQUENCY, BANDWIDTH);
////
//	while(true) {
//		HAL_Delay(1000);
//		char message[32] = "W9FTX radio check W9FTX";
////		RFM96_TransmitAFSK(message, strlen(message));
//		rfm96_send_afsk_data(message, strlen(message), &hspi2, LoRa_CS_GPIO_Port, LoRa_CS_Pin);
//		log_message("send message");
//	}

	heartbeat_entry* radio_entry = calloc(1, sizeof(heartbeat_entry));
	radio_entry->function = radio_heartbeat;
	radio_entry->interval = 1;
	radio_entry->next = null;
	radio_entry->timeUntilNext = 1; // give 3ms so hopefully tasks dont overlap, plus 150 MS so that the BME280 device can take at least one sample
	radio_entry->name = "radio";

	register_heartbeat_func(radio_entry);


	uint8_t data2[4] = {0x00, 0xFF, 0b10101010, 0b11000011};
	radio_enqueue(data2, sizeof(data2));

	telemetry_send("W9FTX radio check");

}


#define TELEMETRY_INTERVAL 8000 // every 7 seconds

static int32_t timer = TELEMETRY_INTERVAL*2;

void telemetry_heartbeat(system_data* data) {
	timer--;
	if (timer <= 0) {
		telemetry_send("W9FTX");
		timer = TELEMETRY_INTERVAL;
		log_message("sending message");
	}
}

/*
 * Make sure data ends with a null character, otherwise program will break
 * Also, size of data must be less than 255
 */
void telemetry_send(const char* data) {
	if (!initOk) {
		log_message("cannot send radio data because initOk is bad");
		return;
	}
	if (strlen(data) > 254) {
		log_message("Trying to send too much data over radio! (at least as of right now), max is 254 characters");
		return;
	}

	radio_enqueue(data, strlen(data));

	//
//
////	strcpy(LoRa_buff, data);
//#ifdef RADIO_FSK_MODE
//	bool txCplt = RF95FSK_send_try4(data, strlen(data));
//#else
//	bool txCplt = RF95_send(data);
//#endif
//
//	if (!txCplt)
//		log_message("RF95 reports transmit error");
//	else
////		;
//		log_message("RF95 reports good transmit");
//
//	HAL_SPI_Receive_IT(&hspi2, RX_Buffer, sizeof(RX_Buffer)); // I think transmitting disables the IRQ/receive so we need to re-enable
}

void telemetry_RxCplt(SPI_HandleTypeDef* spi) {
	HAL_SPI_Receive_IT(&hspi2, RX_Buffer, sizeof(RX_Buffer));
	log_messagef("TELEMETRY RECIEVED: %s", RX_Buffer);
}

