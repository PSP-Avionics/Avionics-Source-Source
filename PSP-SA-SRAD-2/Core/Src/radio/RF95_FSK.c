/*
 * Code taken from https://github.com/MartiGeek17/LoRa-RFM95-STM32
 */

#include "radio/RF95_FSK.h"
#include "main.h"
#include "settings.h"

/*
 *
 * LoRa RFM9x pin mapping
 *
 * LoRa	-> Nucleo
 *
 * Vin	-> 3.3V
 * GND	-> GND
 * EN	-> (disconnected)
 * G0	-> (disconnected)
 * SCK	-> B10
 * MISO	-> C2
 * MOSI	-> C3
 * CS	-> D0
 * RST	-> D1
 *
 *
 */

extern SPI_HandleTypeDef LoRa_SPI;

static RHMode _mode;

struct RF95FSK_settings {
	uint32_t freqLow;
	uint32_t freqHigh;
	uint32_t freqMid;
	uint32_t delay_us_per_bit;
	bool ookMode;
};

static struct RF95FSK_settings inst;

static HAL_StatusTypeDef err;
HAL_StatusTypeDef RF95FSK_write(char reg, char wValue)
{
	char buff[2]={0};
	
	buff[0] = W | reg;
	buff[1] = wValue;

	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	err = HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&buff, 2, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return err;
}


HAL_StatusTypeDef RF95FSK_write_burst(char reg, uint8_t* data, uint16_t length)
{
	uint8_t cmd = W | reg;
	HAL_StatusTypeDef err;

	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&cmd, 1, 100);
	err = HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)data, length, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

	return err;
}


char RF95FSK_read(char reg)
{
	char buff = R & reg;

	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&buff, 1, 100);
	HAL_SPI_Receive(&LoRa_SPI, (uint8_t*)&buff, 1, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return buff;
}


HAL_StatusTypeDef RF95FSK_read_burst(char reg, char* buffer, int length)
{
	buffer[0] = R & reg;
	HAL_StatusTypeDef err;
	
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)buffer, 1, 100);
	err = HAL_SPI_Receive(&LoRa_SPI, (uint8_t*)buffer, length, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return err;
}


void RF95FSK_Reset(void)
{
	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_SET);
}

static uint8_t rbuff = 0;
bool RF95FSK_Init(float freq_mhz, float bw_kHz, uint32_t bitrate)
{
	inst = (struct RF95FSK_settings) {
			.freqLow = calc_frf(freq_mhz*FRF_SCALAR2-bw_kHz*500.0f), // *0 so freq is what it is supposed to be. this is no longer really being used IG
			.freqMid = calc_frf(freq_mhz*FRF_SCALAR2),
			.freqHigh = calc_frf(freq_mhz*FRF_SCALAR2+bw_kHz*500.0f),
			.delay_us_per_bit = 1000000/bitrate,
			.ookMode = false
	};

	RF95FSK_Reset();

	// Set sleep mode, so we can also set LORA mode:
	RF95FSK_sleep();
	
    RF95FSK_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP); // need to update this line of code if ook is true
    HAL_Delay(20); // Wait for sleep mode to take over from say, CAD

    // Check we are in sleep mode, with LORA set
	rbuff = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
    if (rbuff != (RH_RF95_MODE_SLEEP))
    {
    	log_messagef("RF95 error 1 (%d)", rbuff);
    	return false; // No device present?
    }


    RF95FSK_setModeIdle();

    // An innocuous ISM frequency, same as RF22's
    RF95FSK_setFrequency1f(433.0f);
    // Lowish power
    RF95FSK_setTxPower(13, false);

    return true;
}



void RF95FSK_setModemConfigFSK(uint16_t bitrate, uint16_t freqDeviationHz) {
	uint8_t PaRamp = 0b1111;//0b1001;
	uint8_t ModulationShaping = 0b01100000;

//	bitrate = 1200; // im lazy for now

	// undo LoRa settings
//
//	RF95_write(0x1D, 0);
//	RF95_write(0x1E, 0);
//	RF95_write(0x26, 0);

	// FSK settings

	float scalar = 0.999428001369688f;
    uint32_t frf = (uint32_t)((scalar * freqDeviationHz)) / (uint32_t)RH_RF95_FSTEP;
//
//	RF95FSK_write(0x0A, (ModulationShaping) | PaRamp);
//	RF95FSK_write(0x02, (bitrate & 0xFF00) >> 8);
//	RF95FSK_write(0x03, (bitrate & 0xFF));
//	RF95FSK_write(0x04, (frf >> 8) & 0xFF); // two left-most bits are reserved
//	RF95FSK_write(0x05, (frf) & 0xFF);

	// packet config
	bool variableLenPackets = true;
	bool crcOn = false;
	bool CrcAutoClearOff = false;
	uint8_t addressFiltering = 0; // 0 is off
	bool crcWhiteningType = 0;

	bool dataMode = false; // true is packet mode, false is continuous
	bool IoHomeOn = false;
	bool BeaconOn = false;

	uint8_t RegPacketConfig1 = (variableLenPackets << 7) | (crcOn << 4) | (CrcAutoClearOff << 3) | (addressFiltering << 1);
	uint8_t RegPacketConfig2 = (dataMode << 6) | (IoHomeOn << 5) | (BeaconOn << 3);

	RF95FSK_write(0x30, RegPacketConfig1);
	RF95FSK_write(0x31, RegPacketConfig2);
//	RF95FSK_write(0x32, strlen("W9FTX radio check W9FTX")); // registry for pkt_len for fixed packet sizes

	RF95FSK_write(0x35, 0b10000000);

}

bool RF95FSK_send_try3(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();
	RF95FSK_setFrequency1i(inst.freqLow);
	RF95FSK_setModeTx(); // Start the transmitter

	for (int i = 0; i < len; i ++) {
		RF95FSK_write(0x00, data[i]);
	    RF95FSK_setModeTx(); // Start the transmitter
		log_messagef("fifo status: 0x3f & 32: %d 0x3e: %d", (int) RF95FSK_read(0x3e) & 32, (int) RF95FSK_read(0x3f));
		while ( RF95FSK_read(0x3f) == 32);
	}

	/*
	 Preamble (1010...)
 Sync word (Network ID)
 Length byte
 Optional Address byte (Node ID)
 Message data
*/


	HAL_Delay(1000); // idk just fow now

	RF95FSK_setModeIdle();

}

bool RF95FSK_send_try2(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();

    RF95FSK_setModeTx(); // Start the transmitter

	for (int i = 0 ; i < len; i ++)
		data[i] = i;

//	RF95FSK_write_burst(0x00, data, len);

	uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;
	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {

//		uint8_t count = 255;
//		while (!RF95FSK_Check_TxDone() && (count-- != 0));
//		if (count == 0) log_messagef("timed out");
		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] >> (bit)) & 0x1;

            uint16_t freq = 1; // go with something low for now //isHigh ? MARK_FREQ : SPACE_FREQ;
            uint32_t period_us = 1000000 / freq;
            uint32_t half_period_us = period_us / 2;

            delay = period_us;


            if (isHigh)
				RF95FSK_setFrequency1i(inst.freqHigh);
			else
				RF95FSK_setFrequency1i(inst.freqLow); // TODO (if i care enough): only set 1 bit of possible instead of needing 3 cycles/I2C write calls

            RF95FSK_write(0x02, 0xFF);
    		RF95FSK_write(0x03, 0xFF);
			RF95FSK_setModeTx_Continuous(); // Start the transmitter
			RF95FSK_setModeTx(); // Start the transmitter

		    now += delay;
			next += delay;

			while (get_time_us() < next);
		}

		log_messagef("byte %d/%d", data_idx, len);
		HAL_Delay(250);
	}

	uint16_t count = 65536;
	while (!RF95FSK_Check_TxDone() && (count-- != 0));
	if (count == 0) log_message("timed out at end");


//	HAL_Delay(1000);

	RF95FSK_setModeIdle();



    return true;

}

bool RF95FSK_send_try4_old(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();
    RF95FSK_setModeTx(); // Start the transmitter

    uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;

	const uint16_t fdev0 = 64;
	const uint16_t bitrate0 = 4096;
	const uint16_t fdev1 = 512;
	const uint16_t bitrate1 = 8192;

	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {

		RF95FSK_setFrequency1i(inst.freqMid);

		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] >> (bit)) & 0x1;

			// TODO : move these variables here before the first for loop
            uint16_t freq = 8; // go with something low for now //isHigh ? MARK_FREQ : SPACE_FREQ;
            uint32_t period_us = 1000000 / freq;
            uint32_t half_period_us = period_us / 2;

            delay = period_us;

            if (isHigh) {
                RF95FSK_write(0x02, (uint8_t) ((bitrate1 & 0xFF00) >> 8)); // bitrate MSB
        		RF95FSK_write(0x03, (uint8_t) (bitrate1 & 0xFF)); // bitrate LSB
                RF95FSK_write(0x04, (uint8_t) ((fdev1 & 0xFF00) >> 8)); // Fdev MSB
        		RF95FSK_write(0x05, (uint8_t) (fdev1 & 0xFF)); // Fdev LSB
            } else {
                RF95FSK_write(0x02, (uint8_t) ((bitrate0 & 0xFF00) >> 8)); // bitrate MSB
        		RF95FSK_write(0x03, (uint8_t) (bitrate0 & 0xFF)); // bitrate LSB
                RF95FSK_write(0x04, (uint8_t) ((fdev0 & 0xFF00) >> 8)); // Fdev MSB
        		RF95FSK_write(0x05, (uint8_t) (fdev0 & 0xFF)); // Fdev LSB
        	}

			RF95FSK_setModeIdle(); // Start the transmitter
			HAL_Delay(1);
			RF95FSK_setModeTx(); // Start the transmitter
			HAL_Delay(20);
			RF95FSK_setModeIdle(); // Start the transmitter

		    now += delay;
			next += delay;

//			while (get_time_us() < next);
		}

		log_messagef("byte %d/%d", data_idx, len);
		HAL_Delay(250);
	}

	uint16_t count = 65536;
	while (!RF95FSK_Check_TxDone() && (count-- != 0));
	if (count == 0) log_message("timed out at end");

	RF95FSK_setModeIdle();
    return true;
}

#define RFM96_DIO2_GPIO_Port GPIOD
#define RFM96_DIO2_GPIO_Pin GPIO_PIN_12

uint16_t fdev0_override;
uint16_t bitrate0_override;


bool RF95FSK_send_try4(uint8_t* data, uint16_t len)
{
	// TODO : add cooldown of 250ms or something for safety reasons. it's more reliable that way
    RF95FSK_setModeIdle();

    RF95FSK_write(0x25, 0x00);
    RF95FSK_write(0x26, 0x00); // set preamble len to 0
    RF95FSK_write(0x31, 0x00); // set to continuous mode
    RF95FSK_write(0x40, 0x00); // pin mapping
    RF95FSK_write(0x35, 0b10000000); // set preamble len to 0

	const uint16_t fdev0 = 16;
	const uint16_t bitrate0 = 512;

//    uint16_t fdev0 = fdev0_override;
//    uint16_t bitrate0 = bitrate0_override;

	RF95FSK_write(0x02, (uint8_t) ((bitrate0 & 0xFF00) >> 8)); // bitrate MSB
	RF95FSK_write(0x03, (uint8_t) (bitrate0 & 0xFF)); // bitrate LSB
    RF95FSK_write(0x04, (uint8_t) ((fdev0 & 0xFF00) >> 8)); // Fdev MSB
	RF95FSK_write(0x05, (uint8_t) (fdev0 & 0xFF)); // Fdev LSB
	RF95FSK_setFrequency1i(inst.freqMid);
	RF95FSK_write(0x00, 0x00);

	uint8_t mode = 0;


    uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;

	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {

//	    RF95FSK_setModeTx(); // Start the transmitter

		log_messagef("byte %d/%d fdev: %d bitrate: %d", data_idx, len, (int) fdev0, (int) bitrate0);

		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] >> (bit)) & 0x1;

			HAL_GPIO_WritePin(RFM96_DIO2_GPIO_Port, RFM96_DIO2_GPIO_Pin, isHigh); // sets the Vcc for the IO interface on the THP sensor to high

			if (mode == 0) {
				mode = 1;
				RF95FSK_setModeTx(); // Start the transmitter
			}

		    now += delay;
			next += delay;

			while (get_time_us() < next);
		}

	}

	log_messagef("byte %d/%d fdev: %d bitrate: %d", len, len, (int) fdev0, (int) bitrate0);

	RF95FSK_setModeIdle();
    return true;
}

static uint8_t fifo_buffer[255];
static uint8_t fifo_idx;
static uint8_t transmitting_state;

static uint8_t curr_byte;
static uint8_t curr_bit;

static uint32_t next_ms_to_swap;

const uint16_t _fdev = 16;
const uint16_t _bitrate = 512;

bool radio_enqueue(uint8_t* buffer, uint8_t len) {
	if (fifo_idx + len < sizeof(fifo_buffer)) {
		for (int i = 0; i < len; i ++) {
			fifo_buffer[i + fifo_idx] = buffer[i];
		}
		fifo_idx += len;
		return true;
	}
	return false;

}

void radio_heartbeat(system_data* data) {
	if (transmitting_state == 0) {
		// currently off
		if (fifo_idx > 0) {
			curr_byte = fifo_buffer[0];
			for (int i = 1; i <= fifo_idx; i ++) {
				fifo_buffer[i-1] = fifo_buffer[i];
			}
			fifo_idx--;
			curr_bit = 7;

			// begin transmitting
			RF95FSK_setModeIdle();

			// TODO : reset the RFM96 so its at a known state

			RF95FSK_write(0x25, 0x00);
			RF95FSK_write(0x26, 0x00); // set preamble len to 0
			RF95FSK_write(0x31, 0x00); // set to continuous mode
			RF95FSK_write(0x40, 0x00); // pin mapping
			RF95FSK_write(0x35, 0b10000000); // set preamble len to 0

			RF95FSK_write(0x02, (uint8_t) ((_bitrate & 0xFF00) >> 8)); // bitrate MSB
			RF95FSK_write(0x03, (uint8_t) (_bitrate & 0xFF)); // bitrate LSB
		    RF95FSK_write(0x04, (uint8_t) ((_fdev & 0xFF00) >> 8)); // Fdev MSB
			RF95FSK_write(0x05, (uint8_t) (_fdev & 0xFF)); // Fdev LSB
			RF95FSK_setFrequency1i(inst.freqMid);
			RF95FSK_write(0x00, 0x00);

			transmitting_state = 1; // ready to xmit

		}
	} else if (transmitting_state == 1) {
		uint8_t isHigh = (curr_byte >> (curr_bit)) & 0x1;
		HAL_GPIO_WritePin(RFM96_DIO2_GPIO_Port, RFM96_DIO2_GPIO_Pin, isHigh); // sets the Vcc for the IO interface on the THP sensor to high
		RF95FSK_setModeTx(); // Start the transmitter

		next_ms_to_swap = HAL_GetTick() + 1000 / BITRATE - 1; // hardcode for now is ok. add - 1 to adjust for clock cycles, etc
		transmitting_state = 2; // now actively transmitting
	} else if (transmitting_state == 2) {
		if (HAL_GetTick() >= next_ms_to_swap) {
			curr_bit--;

			if (curr_bit == 99) {
				// finish xmit
				RF95FSK_setModeIdle();
				transmitting_state = 0;
			} else {
				uint8_t isHigh = (curr_byte >> (curr_bit)) & 0x1;
				HAL_GPIO_WritePin(RFM96_DIO2_GPIO_Port, RFM96_DIO2_GPIO_Pin, isHigh); // sets the Vcc for the IO interface on the THP sensor to high

				next_ms_to_swap += 1000 / BITRATE;

				if (curr_bit == 0) {
					if (fifo_idx == 0) {
						curr_bit = 100; // no more data left
					} else {
						curr_bit = 8;
						curr_byte = fifo_buffer[0];
						for (int i = 1; i <= fifo_idx; i ++) {
							fifo_buffer[i-1] = fifo_buffer[i];
						}
						fifo_idx--;
					}
				}
			}
		}
	}

	telemetry_heartbeat(data);

}

bool RF95FSK_send_try2_debug(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();

    RF95FSK_setModeTx(); // Start the transmitter

	for (int i = 0 ; i < len; i ++)
		data[i] = i;

//	RF95FSK_write_burst(0x00, data, len);

	uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;

//		uint8_t count = 255;
//		while (!RF95FSK_Check_TxDone() && (count-- != 0));
//		if (count == 0) log_messagef("timed out");
		for (uint32_t bitrate = 0x01; bitrate <= 0xFFFF; bitrate *= 2) {
			for (uint32_t fdev = 0x01; fdev <= 0xFFFF; fdev *= 2) {
//			uint8_t isHigh = (data[data_idx] >> (bit)) & 0x1;

            uint16_t freq = 1; // go with something low for now //isHigh ? MARK_FREQ : SPACE_FREQ;
            uint32_t period_us = 1000000 / freq;
            uint32_t half_period_us = period_us / 2;

            delay = period_us;

            RF95FSK_setFrequency1i(inst.freqMid);
            RF95FSK_write(0x02, (uint8_t) ((bitrate & 0xFF00) >> 8)); // bitrate MSB
    		RF95FSK_write(0x03, (uint8_t) (bitrate & 0xFF)); // bitrate LSB
            RF95FSK_write(0x04, (uint8_t) ((fdev & 0xFF00) >> 8)); // Fdev MSB
    		RF95FSK_write(0x05, (uint8_t) (fdev & 0xFF)); // Fdev LSB
			RF95FSK_setModeIdle(); // Start the transmitter
			RF95FSK_setModeTx(); // Start the transmitter

			log_messagef("bitrate: %d fdev: %d", bitrate, fdev);
			RF95FSK_write_burst(0x00, data, len);

		    now += delay;
			next += delay;

			while (get_time_us() < next);
		}

		HAL_Delay(250);
	}

	uint16_t count = 65536;
	while (!RF95FSK_Check_TxDone() && (count-- != 0));
	if (count == 0) log_message("timed out at end");


//	HAL_Delay(1000);

	RF95FSK_setModeIdle();



    return true;

}

bool RF95FSK_send_try2_works(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();

    RF95FSK_setModeTx(); // Start the transmitter
	RF95FSK_setFrequency1i(inst.freqLow);
	RF95FSK_write(0x0A, 0b00101001);
	RF95FSK_setModemConfigFSK(BITRATE, (uint16_t) (BANDWIDTH*1000));

	for (int i = 0 ; i < len; i ++)
		data[i] = i;

	RF95FSK_write_burst(0x00, data, len);

//	RF95FSK_setFrequency1i(inst.freqLow);
//    RF95FSK_setModeTx(); // Start the transmitter

	uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;
	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {
//		RF95FSK_write(0x00, data[data_idx]);
//		RF95FSK_write(0x00, 0x00);
//		RF95FSK_write(0x00, 0xFF);
		uint8_t count = 255;
		while (!RF95FSK_Check_TxDone() && (count-- != 0));
		if (count == 0) log_messagef("timed out");
		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] >> (bit)) & 0x1;

            uint16_t freq = isHigh ? MARK_FREQ : SPACE_FREQ;
            uint32_t period_us = 1000000 / freq;
            uint32_t half_period_us = period_us / 2;

			uint8_t tx_data[1];
            for (uint32_t k = 0; k < (BAUD_RATE / 8); k++) {
//            	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
//				if (isHigh)
//					tx_data[0] = 0xFF;
//				else
//					tx_data[0] = 0x00;
//				HAL_SPI_Transmit(&hspi2, tx_data, 1, HAL_MAX_DELAY);
//            	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

//				RF95FSK_write(0x00, tx_data); // instead of data[data_idx]
//            	RF95FSK_write(0x00, isHigh?0xFF:0x00); // instead of data[data_idx]


				delay_us(half_period_us);
			}


//        	log_messagef("%d %d", (int) tx_data[0], (int) isHigh);

//			if (isHigh) {
//				RF95FSK_write(0x00, 0xFF);
////				RF95FSK_setFrequency1i(inst.freqLow);
//			} else {
//				RF95FSK_write(0x00, 0x00);
////				RF95FSK_setFrequency1i(inst.freqLow);
//			}


//		    RF95FSK_setModeTx(); // Start the transmitter

//		    log_messagef("delay: %d", next - get_time_us());

//		    int32_t sleepTime = next - get_time_us();
//
//		    if (sleepTime > 0)
//		    	delay_us(sleepTime);

		    now += delay;
			next += delay;
		}

		log_messagef("byte %d/%d", data_idx, len);
	}

	uint16_t count = 65536;
	while (!RF95FSK_Check_TxDone() && (count-- != 0));
	if (count == 0) log_message("timed out at end");


	HAL_Delay(1000);

	RF95FSK_setModeIdle();



    return true;
}


bool RF95FSK_send(uint8_t* data, uint16_t len)
{
    RF95FSK_setModeIdle();

	RF95FSK_setFrequency1i(inst.freqLow);
    RF95FSK_setModeTx(); // Start the transmitter

	uint32_t delay = inst.delay_us_per_bit;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;
	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {
		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] << bit) & 0x1;

			if (isHigh)
				RF95FSK_setFrequency1i(inst.freqHigh);
			else
				RF95FSK_setFrequency1i(inst.freqLow);

		    RF95FSK_setModeTx(); // Start the transmitter


//			delay_us((uint64_t) (next - get_time_us()));
		    delay_us(100000);
			now += delay;
			next += delay;
		}
		log_messagef("byte %d/%d", data_idx, len);
	}

	RF95FSK_setModeIdle();



    return true;
}


void RF95FSK_setPreambleLength(uint16_t bytes)
{
    RF95FSK_write(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    RF95FSK_write(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}


static bool RF95FSK_setFrequency1f(float centre)
{

	HAL_StatusTypeDef err = 0;

    uint64_t frf = calc_frf(centre);
    err = err | RF95FSK_write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    err = err | RF95FSK_write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    err = err | RF95FSK_write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return !err;
}

static bool RF95FSK_setFrequency1i(uint32_t frf)
{
	HAL_StatusTypeDef err = 0;

    err = err | RF95FSK_write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    err = err | RF95FSK_write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    err = err | RF95FSK_write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return !err;
}

void RF95FSK_setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
        if (power > 14)power = 14;
        if (power < -1)power = -1;
        RF95FSK_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    }
    else
    {
        if (power > 23)power = 23;
        if (power < 5)power = 5;

        // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
        // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
        // for 21 and 23dBm
        if (power > 20)
        {
            RF95FSK_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
            power -= 3;
        }
        else
        {
            RF95FSK_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
        }

        // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower.
        // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
        // but OutputPower claims it would be 17dBm.
        // My measurements show 20dBm is correct
        RF95FSK_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    }
}


bool RF95FSK_receive(uint8_t* data)
{
	int len = 0;
	
	if(_mode == RHModeRx)
	{
		while(!RF95FSK_available()){}

		if(RF95FSK_Check_PayloadCRCError())
			return false;
		

		len = RF95FSK_read(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		RF95FSK_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, RF95FSK_read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		
		RF95FSK_read_burst(RH_RF95_REG_00_FIFO, (char*)data, len);

		RF95FSK_setModeIdle();
			
		RF95FSK_Clear_IRQ();

		return true;
	}
	else
		return false;
}



bool RF95FSK_available(void)
{
	while(!RF95FSK_Check_RxDone())
	{
		
	}

	if(RF95FSK_Check_ValidHeader())
	{
			RF95FSK_Clear_IRQ();
			return true;
	}
	else 
		return false;
}


bool RF95FSK_available_Timeout(uint16_t timeout)
{
	unsigned long t = HAL_GetTick();

	while(!RF95FSK_Check_RxDone())
	{
		if (HAL_GetTick() - t > timeout)
			return false;
	}

	if(RF95FSK_Check_ValidHeader())
	{
			RF95FSK_Clear_IRQ();
			return true;
	}
	else
		return false;
}


void RF95FSK_setModeCAD(void)
{
	if (_mode != RHModeCad)
	{
			RF95FSK_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD);
			RF95FSK_write(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
			_mode = RHModeCad;
	}
}

static uint8_t aux = 0;
void RF95FSK_setModeIdle(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeIdle)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_STDBY;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        _mode = RHModeIdle;
    }
}


bool RF95FSK_sleep(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeSleep)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_SLEEP;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        _mode = RHModeSleep;
    }
    return true;
}


void RF95FSK_setModeRx_Single(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_RXSINGLE;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95FSK_write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF95FSK_setModeRx_Continuous(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_RXCONTINUOUS;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95FSK_write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF95FSK_setModeTx(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeTx)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_TX;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95FSK_write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        _mode = RHModeTx;
    }
}

void RF95FSK_setModeTx_Continuous(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeFSTx)
    {
				aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_FSTX;
        RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95FSK_write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        _mode = RHModeFSTx;
    }
}

void RF95FSK_setOpMode(bool loraMode, bool fskMode) {
	aux = RF95FSK_read(RH_RF95_REG_01_OP_MODE) & 0b00011111; // clear bits we are about to change

	if (loraMode) {
		aux = 0x80 | aux;
	} else {
		if (fskMode)
			aux = 0x00 | aux;
		else
			aux = 0b00100000 | aux;
	}

    RF95FSK_write(RH_RF95_REG_01_OP_MODE, aux);
}

bool RF95FSK_Check_RxTimeout(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_RX_TIMEOUT) >> 7;
	
	return reg_read;
}


bool RF95FSK_Check_RxDone(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_RX_DONE) >> 6;
	
	return reg_read;
}


bool RF95FSK_Check_PayloadCRCError(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_PAYLOAD_CRC_ERROR) >> 5;
	
	return reg_read;
}


bool RF95FSK_Check_ValidHeader(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF95_VALID_HEADER) >> 4;

	return reg_read;
}


bool RF95FSK_Check_TxDone(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF95_TX_DONE) >> 3;

	return reg_read;
}

void RF95FSK_Clear_IRQ(void)
{
	uint8_t irq_flags = 0;
	
	RF95FSK_write(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
	
	irq_flags = RF95FSK_read(RH_RF95_REG_12_IRQ_FLAGS);
	if(irq_flags != 0)
		RF95FSK_write(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
}


bool RF95FSK_Check_ModemClear(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_CLEAR) >> 4;
	
	return reg_read;
}


bool RF95FSK_Check_HeaderInfoValid(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_HEADER_INFO_VALID) >> 3;

	return reg_read;
}


bool RF95FSK_Check_RxOnGoing(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_RX_ONGOING) >> 2;

	return reg_read;
}

bool RF95FSK_Check_SignalSyncronized(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED) >> 1;
	
	return reg_read;
}


bool RF95FSK_Check_SignalDetect(void)
{
	char reg_read = 0;
	reg_read = RF95FSK_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_SIGNAL_DETECTED);
	
	return reg_read;
}


//static uint16_t ComputeCRC(uint16_t crc, uint8_t data, uint16_t polynomial)
//{
//	uint8_t i;
//	for(i = 0; i < 8; i++)
//	{
//		if((((crc & 0x8000) >> 8) | (data & 0x80)) != 0)
//		{
//			crc <<= 1;
//			crc |= polynomial;
//		}
//		else
//		{
//			crc <<= 1;
//		}
//		data <<= 1;
//	}
//	return crc;
//}


uint16_t RF95FSK_ComputeCRC(uint8_t *buffer, uint8_t bufferLength, uint8_t crcType)
{
	uint8_t i;
	uint16_t crc;
	uint16_t polynomial;

	polynomial = (crcType == CRC_TYPE_IBM) ? POLYNOMIAL_IBM : POLYNOMIAL_CCITT;
	crc = (crcType == CRC_TYPE_IBM) ? CRC_IBM_SEED : CRC_CCITT_SEED;

	for(i = 0; i < bufferLength; i++)
	{
		crc = ComputeCRC(crc, buffer[i], polynomial);
	}

	if(crcType == CRC_TYPE_IBM)
	{
		return crc;
	}
	else
	{
		return (uint16_t)(~crc);
	}
}




////========================================================================
////=======================Communication protocol===========================
////========================================================================
///* The communication protocol uses three kinds of symbols
// * 		+ '?' -> to make petitions to a certain node
// * 		+ 'O' -> to say something has been done correctly or to continue
// * 		+ 'X' -> to say something has gone wrong or to stop
// * 	All of them are followed with the name of the node, which is only a number.
// */
//
//void Clear_Buffer(uint8_t* buffer)
//{
//	memset(buffer, 0, strlen((const char *)buffer));
//}






