/**
 * @file xl320_driver.c
 *
 *	@author Jean-François Castellani
 *	@author Lucas Vallery
 *
 */
#include "xl320_driver.h"

int xl320_init(XL320_t* xl320, uint8_t id, XL320_BaudRate_t br){
	xl320->id 	= id;
	xl320->br	= br;

	return 0;
}

void xl320_addHeader2Buff(uint8_t* buff){
	buff[0] = 0xFF;
	buff[1] = 0xFF;
	buff[2] = 0xFD;
	buff[3] = 0x00;
}

unsigned short xl320_updateCrc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
	unsigned short i, j;
	unsigned short crc_table[256] = {
			0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
			0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
			0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
			0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
			0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
			0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
			0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
			0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
			0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
			0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
			0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
			0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
			0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
			0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
			0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
			0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
			0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
			0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
			0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
			0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
			0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
			0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
			0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
			0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
			0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
			0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
			0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
			0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
			0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
			0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
			0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
			0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++) {
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

void xl320_copyParams2Buff(uint8_t buffStartIndex, uint8_t* buff, uint16_t nbParams, uint8_t* params){
	for(int i = 0; i < nbParams; i++){
		buff[i + buffStartIndex] = params[i];
	}
}

int xl320_sendCommand(XL320_t* xl320, XL320_Instruction_t inst, uint16_t nbParams, uint8_t* params){
	uint8_t* txBuff = NULL;
	txBuff = (uint8_t*) malloc((MIN_FRAME_SIZE + nbParams)*sizeof(uint8_t));

	if(txBuff == NULL){
		DEBUG_PRINTF("XL320 ERROT : Malloc failed\r\n");
		return -1;
	}

	uint16_t length = nbParams + 3;

	xl320_addHeader2Buff(txBuff);
	txBuff[4] = xl320->id;
	txBuff[5] = (uint8_t) (length & 0xFF);
	txBuff[6] = (uint8_t) (length >> 8);
	txBuff[7] = inst;
	xl320_copyParams2Buff(8, txBuff, nbParams, params);

	uint16_t crc = xl320_updateCrc(0, txBuff, MIN_FRAME_SIZE + nbParams - CRC_FIELD_SIZE);

	txBuff[(MIN_FRAME_SIZE + nbParams) - 2] = (uint8_t) (crc & 0xFF);
	txBuff[(MIN_FRAME_SIZE + nbParams) - 1] = (uint8_t) (crc >> 8);

	xl320->serial.transmit(txBuff, (MIN_FRAME_SIZE + nbParams)*sizeof(uint8_t), 0x1F4);

	free(txBuff);
	return 0;
}

int xl320_receiveCommand(XL320_t* xl320, uint8_t* rxBuff){
	xl320->serial.receive(rxBuff, BUFFER_SERVO_SIZE, 0x1F4);

	return 0;
}

int xl320_checkErrorField(uint8_t errCode){
	switch(errCode){
	case NO_ERROR :
		return 0;
	case RESULT_FAIL :
		DEBUG_PRINTF("XL320 ERROR : Failed to process the sent Instruction Packet\r\n");
		return -1;
	case INSTR_ERROR :
		DEBUG_PRINTF("XL320 ERROR : Undefined Instruction has been used or action has been used without Reg/Write\r\n");
		return -1;
	case CRC_ERROR :
		DEBUG_PRINTF("XL320 ERROR : CRC of the sent Packet does not match\r\n");
		return -1;
	case DATA_RANGE_ERROR :
		DEBUG_PRINTF("XL320 ERROR : Data to be written in the corresponding Address is outside the range of the min/max value\r\n");
		return -1;
	case DATA_LENGTH_ERROR :
		DEBUG_PRINTF("XL320 ERROR : Attempt to write Data that is shorter than the data length of the corresponding Address\r\n");
		return -1;
	case DATA_LIMIT_ERROR :
		DEBUG_PRINTF("XL320 ERROR : Data to be written in the corresponding Address is outside of the Limit value\r\n");
		return -1;
	case ACCESS_ERROR :
		DEBUG_PRINTF("XL320 ERROR : Attempt to write a value in an Address that is Read Only or has not been defined\r\n");
		DEBUG_PRINTF("              Attempt to read a value in an Address that is Write Only or has not been defined\r\n");
		DEBUG_PRINTF("              Attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock)\r\n");
		return -1;
	}
	return 0;
}

int xl320_check_crcField(uint8_t* buffer){
	uint16_t nbParam = buffer[LEN_FRAME_OFFSET] + (buffer[LEN_FRAME_OFFSET + 1]<<8);

	if(nbParam != 0)
		nbParam -= 3;

	uint16_t crcReceived = buffer[MIN_FRAME_SIZE + nbParam - 2] + (buffer[MIN_FRAME_SIZE + nbParam - 1]<<8);
	uint16_t crcComputed = xl320_updateCrc(0, buffer, MIN_FRAME_SIZE + nbParam - 2);

	if(crcReceived == crcComputed)
		return 0;
	else {
		DEBUG_PRINTF("XL320 ERROR : CRC of the received Packet does not matched\r\n");
		return -1;
	}
}

int xl320_ping(XL320_t* xl320){
	char rxBuff[BUFFER_SERVO_SIZE] = {0};

	xl320_sendCommand(xl320, PING, 0, NULL);
	xl320_receiveCommand(xl320, (uint8_t*) &rxBuff);

	printf("SN LSB %d, SN MSB %d, Firm : %d", rxBuff[9], rxBuff[10], rxBuff[11]);

	return 0;
}

int xl320_reboot(XL320_t* xl320){
	char rxBuff[BUFFER_SERVO_SIZE] = {0};

	xl320_sendCommand(xl320, REBOOT, 0, NULL);
	xl320_receiveCommand(xl320, (uint8_t*) &rxBuff);

	if(xl320_check_crcField((uint8_t*)&rxBuff) == -1)
		return -1;

	if(xl320_checkErrorField(rxBuff[ERR_FRAME_OFFSET]) == -1)
		return -1;

	return 0;
}

int xl320_setLedColor(XL320_t* xl320, XL320_Color_t color){
	uint8_t params[3] = {LED, 0, (uint8_t) color};

	xl320_sendCommand(xl320, WRITE, 3, (uint8_t*) &params);

	return 0;
}

int xl320_setGoalPosition(XL320_t* xl320, float goalPositionInDeg){
	uint16_t position = (uint16_t)(goalPositionInDeg/BIT_RESOLUTION_IN_DEG);
	uint8_t params[4] = {POSITION, 0, (uint8_t)(position & 0xFF) , (uint8_t)(position >> 8)};

	xl320_sendCommand(xl320, WRITE, 4, (uint8_t*) &params);

	return 0;
}

int xl320_setSpeed(XL320_t* xl320, float rpm){
	uint16_t speedValue = (uint16_t)(rpm/RESOLUTION_SPEED);
	uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xFF);
	uint8_t lowByte = (uint8_t)(speedValue & 0xFF);

	uint8_t params[4] = {SPEED, 0, lowByte, highByte};
	xl320_sendCommand(xl320, WRITE, 4, (uint8_t*) &params);

	return 0;
}

int xl320_executeAction(XL320_t* xl320){
	xl320_sendCommand(xl320, ACTION, 0, NULL);

	return 0;
}

int xl320_torqueEnable(XL320_t* xl320){
	uint8_t params[3] = {TORQUE_EN, 0, ENABLE};

	xl320_sendCommand(xl320, WRITE, 3, (uint8_t*) &params);

	return 0;
}


int xl320_readCurrent(XL320_t* xl320){
	char rxBuff[READ_BUFFER_SIZE] = {0};

	uint8_t params[4] = {CURRENT, 0, NB_PARAM_READ, 0};

	xl320_sendCommand(xl320, READ, 4, (uint8_t*) &params);
	xl320_receiveCommand(xl320, (uint8_t*) &rxBuff);

	printf("Param1 = %d, Param2 = %d, Param3 = %d, Param4 = %d", rxBuff[9], rxBuff[10], rxBuff[11], rxBuff[12]);

	return 0;
}

int xl320_factoryReboot(XL320_t* xl320, XL320_Reset_Options_t rst_opt){
	char rxBuff[11] = {0};

	uint8_t params[1] = {rst_opt};

	xl320_sendCommand(xl320, FACTORY_RESET, 1, (uint8_t*) &params);
	xl320_receiveCommand(xl320, (uint8_t*) &rxBuff);

	printf("Réinitialisation complétée, ID = 1 et BR = 1M\r\n");
	printf("%d", rxBuff[7]);

	return 0;
}


int xl320_blinbling(XL320_t* xl320){
	xl320_setLedColor(xl320, Off);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Red);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Green);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Yellow);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Blue);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Purple);
	HAL_Delay(100);
	xl320_setLedColor(xl320, Cyan);
	HAL_Delay(100);
	xl320_setLedColor(xl320, White);
	HAL_Delay(100);

	return 0;
}

