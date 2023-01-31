/**
 * @file xl320_driver.h
 *
 *	@author Jean-François Castellani
 *	@author Lucas Vallery
 *
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "usart.h"
#include "uart_half_duplex_driver.h"

/*
 * OPTION FOR DEBUG
 */

#define DEBUG_XL320_ENABLE

#ifdef DEBUG_XL320_ENABLE
	#define DEBUG_PRINTF(x) printf(x)
#else
	#define DEBUG_PRINTF
#endif


/*
 * SERVO CONSTANTS
 */
#define BIT_RESOLUTION_IN_DEG 	0.29
#define RESOLUTION_SPEED 		0.111
#define LIMIT_SPEED 			114
#define GATE_CLOSED 			69
#define BUFFER_SERVO_SIZE				14
#define READ_BUFFER_SIZE		15
#define CURRENT_RESOLUTION		0.001
#define NB_PARAM_READ			4

/*
 * FRAME SIZE
 */
#define MIN_FRAME_SIZE 		10
#define CRC_FIELD_SIZE		2
#define ERR_FRAME_OFFSET	8 //7
#define LEN_FRAME_OFFSET	5


/*
 * BAUD RATES
 */
typedef enum baudRate_struct{
	BR_9600 	=	0,
	BR_57600 	=	1,
	BR_115200 	= 	2,
	BR_1M 		=	3
}XL320_BaudRate_t;


/*
 * INSTRUCTIONS
 */
typedef enum xl320_instruction_struct{
	PING 				=	0x01,
	READ				=	0x02,
	WRITE 				= 	0x03,
	ACTION 				= 	0x05,
	FACTORY_RESET		=	0x06,
	REBOOT 				= 	0x08
}XL320_Instruction_t;


/*
 * ERRORS
 */
typedef enum xl320_error_struct{
	NO_ERROR			=	0x00,
	RESULT_FAIL			=	0x01,
	INSTR_ERROR			=	0x02,
	CRC_ERROR			=	0x03,
	DATA_RANGE_ERROR	=	0x04,
	DATA_LENGTH_ERROR	=	0x05,
	DATA_LIMIT_ERROR	=	0x06,
	ACCESS_ERROR		=	0x07
}XL320_Error_t;

/*
 * REGISTERS
 */
typedef enum xl320_register_struct{
	TORQUE_EN 	= 	0x18,
	LED 		= 	0x19,
	POSITION 	= 	0x1E,
	SPEED 		= 	0x20,
	CURRENT		=	0x29
}XL320_Register_t;

/*
 * FACTORY RESET OPTIONS
 */
typedef enum xl320_reset_option{
	RST_ALL 			= 	0xFF,
	RST_EXCEPT_ID 		= 	0x01,
	RST_EXCEPT_ID_BR 	= 	0x02
}XL320_Reset_Options_t;

/*
 * LED COLORS
 */
typedef enum xl320_color_struct{
	Off,
	Red,
	Green,
	Yellow,
	Blue,
	Purple,
	Cyan,
	White
}XL320_Color_t;

typedef int (* xl320_transmit_t)(uint8_t *pData, uint16_t size, uint32_t timeout);
typedef int (* xl320_receive_t)(uint8_t *pData, uint16_t size, uint32_t timeout);


/**
 * @brief Serial interface structure.
 *
 * It should be initialized with the corresponding functions according to your target.
 * The XL320 is a digital servo. It communicates in half-duplex (single-wire) UART.
 */
typedef struct XL320_serial_struct{
	xl320_transmit_t transmit; 		//!< Should be initialized with the UART-transmit(half-duplex mode) function corresponding to your target
	xl320_receive_t receive;	 	//!< Should be initialized with the UART-reiceive(half-duplex mode) function corresponding to your target
}XL320_Serial_t;

/**
 * @brief XL320 object
 *
 */
typedef struct XL320_struct{
	uint8_t id; 				//!< ID of the XL320. The default ID is 1
	uint8_t br; 				//!< Baude rate of the XL320. The default baud rate is at 1MBaud/s

	XL320_Serial_t serial;		//!< Serial abstraction object (Target dependent)
}XL320_t;

/**
 * @fn xl320_init(XL320_t* xl320,uint8_t id, XL320_BaudRate_t br)
 *
 * @brief Initialize XL320 structure
 *
 * @param xl320 	Instance to be initialized
 * @param id 		Device's ID
 * @param br		Devices's baud-rate
 *
 *@return Returns zero if successful
 */
int xl320_init(XL320_t* xl320,uint8_t id, XL320_BaudRate_t br);

/**
 * @fn xl320_addHeader2Buff(uint8_t* buff)
 *
 * @brief Add an header to a buffer.
 *
 * The frame format follows the DYNAMIXEL PROTOCOL 2.0.
 * The header is build on the following pattern :
 * 			   Header   			| Reserved
 * ---------------------------------|-----------
 * 	0xFF		0xFF		0xFD	|	0x00
 *
 * 	@param buff		The buffer in which the function will add the header. The buffer's size must be more than 4
 */
void xl320_addHeader2Buff(uint8_t* buff);

/**
 *	@fn unsigned short xl320_updateCrc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
 *
 *	@brief Compute the CRC field of a data frame.
 *
 *	@param	crc_accum 		Must be set to zero.
 *	@param 	data_blk_ptr	Pointer in the first element of the data frame.
 *	@param 	data_blk_size	Size of the data frame.
 *
 *	@return The CRC value on 16 bits.
 */
unsigned short xl320_updateCrc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

/**
 *  @fn xl320_copyParams2Buff(uint8_t buffStartIndex, uint8_t* buff, uint16_t nbParams, uint8_t* params)
 *
 *  @brief Copy a table in an other table
 *  The number of element between the start index and the end of buff (the receiving table)
 *  must be greater than the size of params (the buffer to copy).
 *
 *  @param buffStartIndex 	The address from which the params will be copied.
 *  @param buff				The buffer in wich the params will be copied
 *  @param nbParams 	 	The size of the to-copy table.
 *  @param params			The table to copy.
 */
void xl320_copyParams2Buff(uint8_t buffStartIndex, uint8_t* buff, uint16_t nbParams, uint8_t* params);

/**
 * @fn xl320_sendCommand(XL320_t* xl320, uint8_t inst, uint16_t nbParams, uint8_t* params)
 *
 * @brief Send a command to the corresponding XL320.
 * If this function is used according to the DYNAMIXEL PROTOCOLE 2.0 the address field is coded on 1 byte. The the parameters table should looks like the following example.
 * The 0 stands for the most significant bits of the address which are not used.
 *
 * ~~~~~~{.c}
 * uint8_t params[3] = {ADDR, 0, ..., ...}
 * ~~~~~~
 *
 * @param xl320 	XL320 device
 * @param inst	 	Instruction
 * @param nbParams	The number of parameters. Can be 0
 * @param params 	The pointer on the table containing the parameters. Can be NULL
 *
 * @return 			Returns 0 if success and -1 if failed
 */
int xl320_sendCommand(XL320_t* xl320, uint8_t inst, uint16_t nbParams, uint8_t* params);

/**
 * @fn xl320_receiveCommand(XL320_t* xl320, uint8_t* rxBuff)
 *
 * @brief Receive data from the corresponding XL320
 *
 * @param xl320		XL320 device
 * @param rxBuff	Buffer in which the receive data will be copied
 */
int xl320_receiveCommand(XL320_t* xl320, uint8_t* rxBuff);

int xl320_checkErrorField(uint8_t errCode);

int xl320_check_crcField(uint8_t* buffer);

int xl320_ping(XL320_t* xl320);

int xl320_reboot(XL320_t* xl320);

int xl320_setLedColor(XL320_t* xl320, XL320_Color_t color);

/**
 * @fn xl320_setGoalPosition(XL320_t* xl320, float goalPositionInDeg)
 *
 * @brief Set the position to be reached by the corresponding XL320
 *
 * @param xl320					XL320 device
 * @param goalPositionInDeg		The position to be reached in degree
 */

int xl320_setGoalPosition(XL320_t* xl320, float goalPositionInDeg);

/**
 * @fn xl320_setSpeed(XL320_t* xl320, float rpm)
 *
 * @brief Set the speed to be reached by the corresponding XL320
 *
 * @param xl320			XL320 device
 * @param rpm			Rotational speed in revolutions per minute
 */

int xl320_setSpeed(XL320_t* xl320, float rpm);

/**
 * @fn xl320_executeAction(XL320_t* xl320)
 *
 * @brief Instruction that executes the paquet that has been registrated using the Reg Write Instruction
 *
 * @param xl320			XL320 device
 */

int xl320_executeAction(XL320_t* xl320);

/**
 * @fn xl320_torqueEnable(XL320_t* xl320)
 *
 * @brief Enables the torque by writing a 1 in the register
 *
 * @param xl320			XL320 device
 */

int xl320_torqueEnable(XL320_t* xl320);

/**
 * @fn xl320_readCurrent(XL320_t* xl320)
 *
 * @brief Reading the current at a given moment
 * The value of the current is obtained on 2 bits which must be converted to obtain a usable measurement
 *
 * @param xl320			XL320 device
 *
 * @return current		Return the value of the current
 */

int xl320_readCurrent(XL320_t* xl320);

/**
 * @fn xl320_factoryReboot(XL320_t* xl320)
 *
 * @brief Instruction that resets the Control Table to its initial factory default settings which are 1 for the ID number and 1M for the Baud Rate.
 *
 * @param xl320			XL320 device
 * @param rst_opt		Option for the reset
 */

int xl320_factoryReboot(XL320_t* xl320, XL320_Reset_Options_t rst_opt);

int xl320_blinbling(XL320_t* xl320);

/*
void xl320_clearReceiveBuffer(uint8_t* buffer);

void xl320_addHeader(void);

void xl320_sendCommand(const uin8_t servoId, uint8_t* packet, uint16_t packetLength);

void xl320_ping(const uint8_t servoId);



//void xl320_led(const uint8_t servoID);

 */
