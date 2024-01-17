/*
 * Code taken from https://github.com/MartiGeek17/LoRa-RFM95-STM32
 */

#include "radio/RF95.h"
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
uint16_t _cad_timeout = 10000;
uint8_t LoRa_buff[RH_RF95_FIFO_SIZE] = {0};

#if Header == Header_used
char _txHeaderTo = 0;
#endif

static char MODEM_CONFIG_TABLE[5][3] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096
		{ 0x73,   0x74,    0x00}, // IH_Bw125Cr45Sf128 (the chip default + Implicit header)
};

static HAL_StatusTypeDef err;
HAL_StatusTypeDef RF95_write(char reg, char wValue)
{
	char buff[2]={0};
	
	buff[0] = W | reg;
	buff[1] = wValue;

	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	err = HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&buff, 2, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return err;
}


HAL_StatusTypeDef RF95_write_burst(char reg, uint8_t* data)
{
	int length = 0;
	uint8_t cmd = W | reg;
	HAL_StatusTypeDef err;
	
	length = strlen((const char*)data);
	
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&cmd, 1, 100);
	err = HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)data, length, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return err;
}


char RF95_read(char reg)
{
	char buff = R & reg;

	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)&buff, 1, 100);
	HAL_SPI_Receive(&LoRa_SPI, (uint8_t*)&buff, 1, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return buff;
}


HAL_StatusTypeDef RF95_read_burst(char reg, char* buffer, int length)
{
	buffer[0] = R & reg;
	HAL_StatusTypeDef err;
	
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI, (uint8_t*)buffer, 1, 100);
	err = HAL_SPI_Receive(&LoRa_SPI, (uint8_t*)buffer, length, 100);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	
	return err;
}


void RF95_Reset(void)
{
	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_SET);
}

static uint8_t rbuff = 0;
bool RF95_Init(bool loraMode)
{
	RF95_Reset();

	// Set sleep mode, so we can also set LORA mode:
	RF95_sleep();
	
    RF95_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | (loraMode?RH_RF95_LONG_RANGE_MODE:0));
    HAL_Delay(20); // Wait for sleep mode to take over from say, CAD

    // Check we are in sleep mode, with LORA set
	rbuff = RF95_read(RH_RF95_REG_01_OP_MODE);
    if (rbuff != (RH_RF95_MODE_SLEEP | (loraMode?RH_RF95_LONG_RANGE_MODE:0)))
    {
    	log_messagef("RF95 error 1 (%d)", rbuff);
    	return false; // No device present?
    }


    if (loraMode) {
		// Set up FIFO
		// We configure so that we can use the entire 256 byte FIFO for either receive
		// or transmit, but not both at the same time
		RF95_write(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
		RF95_write(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
    }

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    RF95_setModeIdle();

    if (loraMode) {
		// Set up default configuration
		// No Sync Words in LORA mode.
		RF95_setModemConfig(Bw125Cr45Sf128); // Radio default
		RF95_setPreambleLength(8); // Default is 8
    }
    // An innocuous ISM frequency, same as RF22's
    RF95_setFrequency(433.0);
    // Lowish power
    RF95_setTxPower(13, false);

    return true;
}


bool RF95_setModemConfig(ModemConfigChoice index)
{
	RF95_write(RH_RF95_REG_1D_MODEM_CONFIG1, MODEM_CONFIG_TABLE[index][0]);
	RF95_write(RH_RF95_REG_1E_MODEM_CONFIG2, MODEM_CONFIG_TABLE[index][1]);
	RF95_write(RH_RF95_REG_26_MODEM_CONFIG3, MODEM_CONFIG_TABLE[index][2]);

    return true;
}

void RF95_setModemConfig1bw(ModemBandwidth bw) {
	uint8_t val_reg_1d = 0b00000010 | (bw << 4);
	uint8_t val_reg_1e = 0b01110000; // 100 for CRC on but I dont want CRC
	uint8_t val_reg_26 = 0x00;

	RF95_write(RH_RF95_REG_1D_MODEM_CONFIG1, val_reg_1d);
	RF95_write(RH_RF95_REG_1E_MODEM_CONFIG2, val_reg_1e);
	RF95_write(RH_RF95_REG_26_MODEM_CONFIG3, val_reg_26);

}

void RF95_setModemConfigFSK(uint16_t bitrate, uint16_t freqDeviationHz) {
	uint8_t PaRamp = 0b1001;//0b1001;
	uint8_t ModulationShaping = 0b00;

	// undo LoRa settings
//
//	RF95_write(0x1D, 0);
//	RF95_write(0x1E, 0);
//	RF95_write(0x26, 0);

	// FSK settings

	float scalar = 0.999428001369688f;
    uint32_t frf = (uint32_t)((scalar * freqDeviationHz)) / (uint32_t)RH_RF95_FSTEP;
//
//	RF95_write(0x0A, (ModulationShaping << 5) | PaRamp);
	RF95_write(0x02, (bitrate & 0xFF00) >> 8);
	RF95_write(0x03, (bitrate & 0xFF));
	RF95_write(0x04, (frf >> 8) & 0xFF); // two left-most bits are reserved
	RF95_write(0x05, (frf) & 0xFF);

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

	RF95_write(0x30, RegPacketConfig1);
	RF95_write(0x31, RegPacketConfig2);
	RF95_write(0x32, strlen("W9FTX radio check W9FTX")); // registry for pkt_len for fixed packet sizes

	RF95_write(0x35, 0b10000000);
}


bool RF95_send_fskDirect(uint8_t* data, uint8_t len, uint16_t bitrate, float freqMidMHz, float offsetHz) {
	float offsetMHz = offsetHz / 1000000.0f;

	float frfMid = freqMidMHz;
	float frfHigh = freqMidMHz + offsetMHz/2.0;
	float frfLow = freqMidMHz - offsetMHz/2.0;

	RF95_setFrequency(frfMid);
    RF95_setModeTx(); // Start the transmitter

	uint32_t delay = 1000000000 / bitrate;

	uint32_t now = get_time_us();
	uint32_t next = now + delay;
	for (uint16_t data_idx = 0; data_idx < len; data_idx++) {
		for (uint8_t bit = 7; bit != 0xFF; bit--) {
			uint8_t isHigh = (data[data_idx] << bit) & 0x1;

			if (isHigh)
				RF95_setFrequency(frfHigh);
			else
				RF95_setFrequency(frfLow);

			delay_us((uint64_t) (next - get_time_us()));
			now += delay;
			next += delay;
		}
	}

	RF95_setModeIdle();


	return true;
}


bool RF95_send(uint8_t* data)
{
	int len = strlen((char*)data);

	#if Header == Header_used
	uint16_t header_len = sizeof(_txHeaderTo);
	#endif

    if (len > RH_RF95_MAX_MESSAGE_LEN)
			return false;

//    RF95_waitPacketSent(); // Make sure we dont interrupt an outgoing message
    RF95_setModeIdle();

//	if (!RF95_waitCAD())
//		return false;  // Check channel activity

//	RF95_setModeIdle();

    // Position at the beginning of the FIFO
//    RF95_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);

//    // The headers
//	#if Header == Header_used
//	RF95_write(RH_RF95_REG_00_FIFO, _txHeaderTo);
//	#endif

    // if var len packets
    uint8_t pktLen = strlen(data);

//    RF95_write(RH_RF95_REG_00_FIFO, pktLen); // uncomment for variable-len pkts

    // set reg value
//    RF95_write(0x35, 0b10000000 | (RF95_read(0x35) & 0x7F));

    // The message data
    for (int i = 0; i < pktLen; i ++) {
    	RF95_write(RH_RF95_REG_00_FIFO, data[i]);
    }
//        	RF95_write_burst(RH_RF95_REG_00_FIFO, data);

//	#if Header == No_header
//	RF95_write(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
//	#else
//	RF95_write(RH_RF95_REG_22_PAYLOAD_LENGTH, len + header_len);
//	#endif

//		uint8_t rBuff[15] = {0};
//		uint16_t lenght_ = RF95_read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);
//		RF95_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, lenght_);
//		RF95_read_burst(RH_RF95_REG_00_FIFO,(char*)rBuff, len);

    RF95_setModeTx(); // Start the transmitter
//    uint8_t attempts = 20;
//	while(!RF95_Check_TxDone()&&attempts!=0){HAL_Delay(1);attempts--;} // wait while mode == tx instead of checking IRQ flag
//
//	if (attempts == 0)
//		log_messagef("RF waited too long for TX to be complete");
//

    uint8_t tgt_mode = RF95_read(RH_RF95_REG_01_OP_MODE) & 0xF8 + 3;

    uint8_t mode;

    while ((mode = (RF95_read(RH_RF95_REG_01_OP_MODE) & 0x07)) != 0b001) {
    	if (mode == 2 || mode == 7)
    		RF95_write(RH_RF95_REG_01_OP_MODE, tgt_mode);
    	log_messagef("mode: %d", mode);
    } // while in Tx Mode. I think it automatically goes into sleep mode after tx done

	RF95_setModeIdle();
//	RF95_Clear_IRQ();
    return true;
}


void RF95_setPreambleLength(uint16_t bytes)
{
    RF95_write(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    RF95_write(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}


bool RF95_setFrequency(float centre)
{

	HAL_StatusTypeDef err = 0;

    // Frf = FRF / FSTEP
	float scalar = 999428.001369688f;// supposed to be 1000000.0 to get MHz -> Hz but the frequency setting gave 445.805 instead of 445.500 like the input so adjusting with this number so 445.500 outputs 445.500 like expected
    uint64_t frf = (uint32_t)((uint32_t)(centre * scalar)) / (uint32_t)RH_RF95_FSTEP;
    err = err | RF95_write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    err = err | RF95_write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    err = err | RF95_write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return !err;
}


void RF95_setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
        if (power > 14)power = 14;
        if (power < -1)power = -1;
        RF95_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
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
            RF95_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
            power -= 3;
        }
        else
        {
            RF95_write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
        }

        // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower.
        // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
        // but OutputPower claims it would be 17dBm.
        // My measurements show 20dBm is correct
        RF95_write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    }
}


bool RF95_receive(uint8_t* data)
{
	int len = 0;
	
	if(_mode == RHModeRx)
	{
		while(!RF95_available()){}

		if(RF95_Check_PayloadCRCError())
			return false;
		

		len = RF95_read(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		RF95_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, RF95_read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		
		RF95_read_burst(RH_RF95_REG_00_FIFO, (char*)data, len);	

		RF95_setModeIdle();
			
		RF95_Clear_IRQ();

		return true;
	}
	else
		return false;
}


bool RF95_receive_Timeout(uint8_t* buf, uint16_t timeout)
{
	int len = 0;

	if(_mode == RHModeRx)
	{
		if(!RF95_available_Timeout(timeout))
		{
			return false;
		}

		if(RF95_Check_PayloadCRCError())
			return false;


		len = RF95_read(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		RF95_write(RH_RF95_REG_0D_FIFO_ADDR_PTR, RF95_read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));

		RF95_read_burst(RH_RF95_REG_00_FIFO, (char*)buf, len);

		RF95_setModeIdle();

		RF95_Clear_IRQ();

		if(RF95_Check_PayloadCRCError())
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	else 
		return false;
}



bool RF95_waitCAD(void)
{
	if (!_cad_timeout)
		return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
			
	RF95_setModeCAD();
		
    unsigned long t = HAL_GetTick();
    while (!RF95_Check_CADDone())
    {
      if (HAL_GetTick() - t > _cad_timeout) 
	     return false;
    }

    return true;
}


bool RF95_waitPacketSent(void)
{
    if (_mode == RHModeTx)
    {
    	while(!RF95_Check_TxDone());
    }

    return true;
}


bool RF95_available(void)
{	
	while(!RF95_Check_RxDone())
	{
		
	}

	if(RF95_Check_ValidHeader())
	{
			RF95_Clear_IRQ();
			return true;
	}
	else 
		return false;
}


bool RF95_available_Timeout(uint16_t timeout)
{
	unsigned long t = HAL_GetTick();

	while(!RF95_Check_RxDone())
	{
		if (HAL_GetTick() - t > timeout)
			return false;
	}

	if(RF95_Check_ValidHeader())
	{
			RF95_Clear_IRQ();
			return true;
	}
	else
		return false;
}


void RF95_setModeCAD(void)
{
	if (_mode != RHModeCad)
	{
			RF95_write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD);
			RF95_write(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
			_mode = RHModeCad;
	}
}

static uint8_t aux = 0;
void RF95_setModeIdle(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeIdle)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_STDBY;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        _mode = RHModeIdle;
    }
}


bool RF95_sleep(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeSleep)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_SLEEP;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        _mode = RHModeSleep;
    }
    return true;
}


void RF95_setModeRx_Single(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_RXSINGLE;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95_write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF95_setModeRx_Continuous(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_RXCONTINUOUS;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95_write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF95_setModeTx(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeTx)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_TX;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95_write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        _mode = RHModeTx;
    }
}

void RF95_setModeTx_Continuous(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeFSTx)
    {
				aux = RF95_read(RH_RF95_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF95_MODE_FSTX;
        RF95_write(RH_RF95_REG_01_OP_MODE, aux);
        RF95_write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        _mode = RHModeFSTx;
    }
}

void RF95_setOpMode(bool loraMode, bool fskMode) {
	aux = RF95_read(RH_RF95_REG_01_OP_MODE) & 0b00011111; // clear bits we are about to change

	if (loraMode) {
		aux = 0x80 | aux;
	} else {
		if (fskMode)
			aux = 0x00 | aux;
		else
			aux = 0b00100000 | aux;
	}

    RF95_write(RH_RF95_REG_01_OP_MODE, aux);
}

bool RF95_Check_RxTimeout(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_RX_TIMEOUT) >> 7;
	
	return reg_read;
}


bool RF95_Check_RxDone(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_RX_DONE) >> 6;
	
	return reg_read;
}


bool RF95_Check_PayloadCRCError(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_PAYLOAD_CRC_ERROR) >> 5;
	
	return reg_read;
}


bool RF95_Check_ValidHeader(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF95_VALID_HEADER) >> 4;

	return reg_read;
}


bool RF95_Check_TxDone(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF95_TX_DONE) >> 3;

	return reg_read;
}


bool RF95_Check_CADDone(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_CAD_DONE) >> 2;

	return reg_read;
}

bool RF95_Check_FhssChannelChange(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_FHSS_CHANGE_CHANNEL) >> 1;
	
	return reg_read;
}


bool RF95_Check_CADDetect(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF95_CAD_DETECTED);
	
	return reg_read;
}


void RF95_Clear_IRQ(void)
{
	uint8_t irq_flags = 0;
	
	RF95_write(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
	
	irq_flags = RF95_read(RH_RF95_REG_12_IRQ_FLAGS);
	if(irq_flags != 0)
		RF95_write(RH_RF95_REG_12_IRQ_FLAGS, 0xFF);
}


bool RF95_Check_ModemClear(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_CLEAR) >> 4;
	
	return reg_read;
}


bool RF95_Check_HeaderInfoValid(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_HEADER_INFO_VALID) >> 3;

	return reg_read;
}


bool RF95_Check_RxOnGoing(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_RX_ONGOING) >> 2;

	return reg_read;
}

bool RF95_Check_SignalSyncronized(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED) >> 1;
	
	return reg_read;
}


bool RF95_Check_SignalDetect(void)
{
	char reg_read = 0;
	reg_read = RF95_read(RH_RF95_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF95_MODEM_STATUS_SIGNAL_DETECTED);
	
	return reg_read;
}


uint16_t ComputeCRC(uint16_t crc, uint8_t data, uint16_t polynomial)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if((((crc & 0x8000) >> 8) | (data & 0x80)) != 0)
		{
			crc <<= 1;
			crc |= polynomial;
		}
		else
		{
			crc <<= 1;
		}
		data <<= 1;
	}
	return crc;
}


uint16_t RF95_ComputeCRC(uint8_t *buffer, uint8_t bufferLength, uint8_t crcType)
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




//========================================================================
//=======================Communication protocol===========================
//========================================================================
/* The communication protocol uses three kinds of symbols
 * 		+ '?' -> to make petitions to a certain node
 * 		+ 'O' -> to say something has been done correctly or to continue
 * 		+ 'X' -> to say something has gone wrong or to stop
 * 	All of them are followed with the name of the node, which is only a number.
 */

void Clear_Buffer(uint8_t* buffer)
{
	memset(buffer, 0, strlen((const char *)buffer));
}

bool RF95_Master_Receive_from_Node(uint16_t node)
{
	uint8_t command[3] = {0};
	uint8_t ACK_command[3] = {0};
	bool error = true; //No error happend
	bool end_flag = 0;

	//Prepare the ACK command
	sprintf((char*)ACK_command, "O%d", node);

	//We prepare the command to send, in this case a request for receiving from the master -> "?X"
	//where the X is the node ID
	sprintf((char*)command, "?%d", node);
	//Request a reception to the node X
	do
	{
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(2000);	//Wait to give time to the slave to change to receive mode
		strcpy((char*)LoRa_buff, (char*)command);
		RF95_send(LoRa_buff);		//Send the request "?x"
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

		while(strcmp((char*)LoRa_buff, (char*)ACK_command) != 0)
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF95_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF95_setModeRx_Continuous();
				error = RF95_receive_Timeout(LoRa_buff, 7000); //Receive an answer and if we doesn't receive it in the time
															   //we send again the petition.
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);


	//Prepare the command Xx in order to check if the transaction has ended or has to stop
	sprintf((char*)command, "X%d", node);
	//Clear the LoRa buffer used for transactions to prevent errors

	Clear_Buffer(LoRa_buff);
	Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	HAL_Delay(3000);	//Wait to give time to the slave to change to receive mode
	strcpy((char*)LoRa_buff, (char*)ACK_command);
	RF95_send(LoRa_buff);		//Send the ACK to indicate the slave to send the data
	Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	Clear_Buffer(LoRa_buff);

	//Receive the data from the node x
	while(end_flag == 0)
	{
		do
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF95_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF95_setModeRx_Continuous();
				error = RF95_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);

				if(strcmp((char *)LoRa_buff, (char *)command) == 0)
				{
					end_flag = 1;
					break;
				}
			}
			/*=====Insert here the code for processing the data received=====*/

			if(strcmp((char*)LoRa_buff, "This is text message!") == 0)
			{
				Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				HAL_Delay(3000);
				Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
			/*===============================================================*/
			if(error == true)
			{
				//Send ACK to the node x
				Clear_Buffer(LoRa_buff);
				Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				HAL_Delay(2000);	//Wait to give time to the slave to change to receive mode
				strcpy((char*)LoRa_buff, (char *)ACK_command); //Send the ACK indicating we want to continue the transmission
				RF95_send(LoRa_buff);
				Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
			}
		} while(error == false);
	}

	Clear_Buffer(LoRa_buff);
	return true;
}


bool RF95_Master_Receive_from_All_Nodes(void)
{
	for(int i = 0; i < Num_of_Nodes; i++)
	{
		RF95_Master_Receive_from_Node(i + 1);
	}
	return true;
}


bool RF95_Slave_Send(void)
{
	uint8_t command[3] = {0};
	uint8_t ACK_command[3] = {0};
	bool error = true; //No error happend

	//Prepare the ACK command
	sprintf((char*)ACK_command, "O%d", Node_ID);


	//We prepare the command to send, in this case a request for receiving from the master -> "?x"
	//where the x is the node ID
	sprintf((char*)command, "?%d", Node_ID);
	//Wait until a receive request is received from Master
	while(strcmp((char *)LoRa_buff, (char *)command) != 0)
	{
		//Receive the data
		Clear_Buffer(LoRa_buff);
		if(RF95_waitCAD())
		{
			Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			RF95_setModeRx_Continuous();
			RF95_receive(LoRa_buff);
		}
	}
	HAL_Delay(1000);
	Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);


	//If a receive request has happened from the Master, we send the ACK to tell the master we are going to send as
	//soon as he send us the ACK
	do
	{
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(2000);
		strcpy((char*)LoRa_buff, (char *)ACK_command);
		RF95_send(LoRa_buff);
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		Clear_Buffer(LoRa_buff);
		//Wait until a receive The ACK from master, which tells us to start sending data
		while(strcmp((char *)LoRa_buff, (char *)ACK_command) != 0)
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF95_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF95_setModeRx_Continuous();
				error = RF95_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);

	/*=====Insert here the code for send data / processing the data received=====*/

	do
	{
		//Send data to the master
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(3000);
		strcpy((char*) LoRa_buff, "This is text message!");
		RF95_send(LoRa_buff);
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

		//Check for ACK
		Clear_Buffer(LoRa_buff);
		while(strcmp((char *)LoRa_buff, (char *)ACK_command) != 0)
		{
			//Receive the data
			if(RF95_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF95_setModeRx_Continuous();
				error = RF95_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(2000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);


	/*===============================================================*/
	//Prepare the command Xx in order to tell the transaction has ended or has to stop
	Clear_Buffer(LoRa_buff);
	Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	sprintf((char*)LoRa_buff, "X%d", Node_ID);
	HAL_Delay(2000);
	RF95_send(LoRa_buff);
	Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

	Clear_Buffer(LoRa_buff);
	return true;
}


