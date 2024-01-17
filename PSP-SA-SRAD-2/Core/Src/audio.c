/*
 * audio.c
 *
 *  Created on: Jun 30, 2022
 *      Author: Alex Bowman
 *
 * Code responsible for playing sounds on the pad so we can know the
 * status of the flight computer
 */

#include "audio.h"
#include "main.h"
#include "flight.h"
#include <pthread.h>

#ifndef DEBUG
#define set_next_beat(delay) ms_next_beat = HAL_GetTick() + delay
#else
#define set_next_beat(delay) ms_next_beat = HAL_GetTick()
#endif

/*
 * Format:
 * bits 0-2: len
 * last x bits: 0 for dot 1 for dash
 */
static const uint8_t morse_table[] = {
		0b01000001, // a .-
		0b10001000, // b -...
		0b10001010, // c -.-.
		0b01100100, // d -..
		0b00100000, // e .
		0b10000010, // f ..-.
		0b01100110, // g --.
		0b10000000, // h ....
		0b01000000, // I ..
		0b10000111, // j .---
		0b01100101, // k -.-
		0b10000100, // l .-..
		0b01000011, // m --
		0b01000010, // n -.
		0b01100111, // o ---
		0b10000110, // p .--.
		0b10001101, // q --.-
		0b01100010, // r .-.
		0b01100000, // s ...
		0b00100001, // t -
		0b01100001, // u ..-
		0b10000001, // v ...-
		0b01100011, // w .--
		0b10001001, // x -..-
		0b10001011, // y -.--
		0b10001100, // z --..
};

static char* ms_message;
static uint8_t ms_idx, ms_char_beep_idx;
static uint32_t ms_next_beat;

static void morse_beep();
static void morse_dash();
static void tune(uint32_t,uint32_t);
static void audio_heartbeat(system_data* data);
void* audio_thread(void *arg);

void init_audio(system_data* data) {

	morse_code("init");

	heartbeat_entry* audio_entry = calloc(1, sizeof(heartbeat_entry));
	audio_entry->function = audio_heartbeat;
	audio_entry->interval = 17;
	audio_entry->next = null;
	audio_entry->timeUntilNext = 0; // give 3ms so hopefully tasks dont overlap, plus 150 MS so that the BME280 device can take at least one sample
	audio_entry->name = "audio";

	register_heartbeat_func(audio_entry);

}

/**
 * NOTE: requires message to be all lowercase, and no numbers
 */
void morse_code(char* message) {
	ms_message = malloc(strlen(message)+1);
	strcpy(ms_message, message);

	ms_idx = 0;
	ms_char_beep_idx = 255;
	ms_next_beat = HAL_GetTick() - 1;

	if (flight_data().state == INIT) {
		while (ms_message != null)
			audio_heartbeat(null);
	}
}

/**
 * TODO : make this function take less time (the tune function has a lot of delays in it)
 */
static void audio_heartbeat(system_data* data) {
	if (ms_message == null)
		return;
	if (HAL_GetTick() < ms_next_beat)
		return;

	flight_state_t state = flight_data().state;
	if (state != INIT && state != PAD && state != LANDED)
		return;

	if (ms_message[ms_idx] != '\0') {
		if (ms_message[ms_idx] - 'a' < 26) {
			uint8_t code = morse_table[ms_message[ms_idx] - 'a'];
			uint8_t len = (code >> 5) & 0b111;

			if (ms_char_beep_idx == 255)
				ms_char_beep_idx = len-1;

			if (code & (1 << ms_char_beep_idx)) {
				morse_dash();
			} else {
				morse_beep();
			}

			if (ms_char_beep_idx == 0) {
				ms_idx++;
				ms_char_beep_idx = 255;
				set_next_beat(500);
			} else {
				ms_char_beep_idx--;
				set_next_beat(200);
			}
		} else {
			set_next_beat(500);
		}
	} else {
		free(ms_message);
		ms_message = null;
	}

}

static void morse_beep() {
	tune(4700,100000);
//	HAL_Delay(200);
}

static void morse_dash() {
	tune(4700,250000);
//	HAL_Delay(200);
}

void* audio_thread(void *arg) {
	while (1) {
		morse_code("hi");

		HAL_Delay(5000);
	}

	return null;
}

// play a tune at 50% duty cycle at a given frequency
// basically the same idea as the arduino tune function
static void tune(uint32_t freq, uint32_t duration_us) {
#ifndef DEBUG // if debug is defined, then mute the annoying speaker

	uint32_t delayUs = 1000000/freq; // i know this is a bad way to do it but it works for now.
	uint32_t count = duration_us/delayUs; // i know this is a bad way to do it but it works for now. Plus, we dont need accuracy

	for (uint32_t i = 0; i < count; i ++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		delay_us(delayUs/2);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		delay_us(delayUs/2);
	}
#endif
}

