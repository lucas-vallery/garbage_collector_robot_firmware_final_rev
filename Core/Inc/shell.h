/*
 * shell.h
 *
 *  Created on: 7 juin 2019
 *      Author: laurent
 */

#ifndef INC_LIB_SHELL_SHELL_H_
#define INC_LIB_SHELL_SHELL_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"


#define ARGC_MAX 					8
#define BUFFER_SIZE 				64
#define SHELL_FUNC_LIST_MAX_SIZE 	32

#define SHELL_TASK_PRIORITY			10
#define STACK_DEPTH					500

struct h_shell_struct;

typedef uint8_t (* drv_uart_shell_transmit_t)(char * pData, uint16_t size);
typedef uint8_t (* drv_uart_shell_receive_t)(char * pData, uint16_t size);

/**
 * @struct drv_uart_shell_t
 * @brief Structure to handle the hardware abstraction layer of the UART communication
 */
typedef struct{
	drv_uart_shell_transmit_t transmit;
	drv_uart_shell_receive_t receive;
} drv_uart_shell_t;


/**
 * @struct 	shell_func_t
 * @brief 	Command structure.
 *
 * @warning The description length should not exceed BUFFER_SIZE characters
 */
typedef struct{
	char * c;															/*!> Command string*/
	int (* func)(struct h_shell_struct* shell, int argc, char ** argv);	/*!> Function to call when the command is executed*/
	char * description;													/*!> A short description of the command*/
} shell_func_t;

/**
 * @struct h_shell_t
 * @brief Shell structure.
 *
 * The functions encapsulated by the drv_uart_shell_t should be initialized with
 * yours implementations of the UART hardware abstraction functions.
 * This initialization could be made as follow :
 * @code{c]
 * shell.uart.receive = drv_uart_receive;
 * shell.uart.transmit = drv_uart_transmit;
 * @endcode
 *
 * @warning Do not add more than SHELL_FUNC_LIST_MAX_SIZE commands to the shell.
 */
typedef struct h_shell_struct{
	int shell_func_list_size;
	char print_buffer[BUFFER_SIZE];

	shell_func_t 		shell_func_list[SHELL_FUNC_LIST_MAX_SIZE];
	drv_uart_shell_t	uart;

	BaseType_t xReturned;
	TaskHandle_t shellTaskHandler;
}h_shell_t;

/**
 * @brief Initialize a h_shell_t structure with default parameters.
 *
 * Besides the initialization of the structure, this function sends the shell's header
 * to the terminal and adds the help and stats commands to the path.
 *
 * @param shell Pointer on h_shell_t structure.
 */
void sh_init(h_shell_t* shell);

/**
 * @brief Add a function to the path.
 *
 * The function need to follow this prototype :
 * @code{c}
 * int (* pfunc)(h_shell_t* shell, int argc, char ** argv)
 * @endcode
 *
 * @param shell 		Pointer on h_shell_t structure.
 * @param c				Command name string.
 * @param pfunc			Function to call on the execution of the command.
 * @param descriprion 	A short description of the command.
 */
int sh_add(h_shell_t* shell, char * c, int (* pfunc)(h_shell_t* shell, int argc, char ** argv), char * description);

/**
 * @brief Create the task in which the shell program will run
 * The task is created with a priority og SHELL_TASK_PRIOTY (10 by default)
 *
 * @warning This function involve a task creation. It may failed.
 *
 * @param Pointer on h_shell_t structure.
 */
int sh_run(h_shell_t* shell);

/**
 * @brief Create a task
 *
 * @param Pointer on h_shell_t structure.
 */
void sh_start(h_shell_t* shell);

/**
 * @brief Call the receive function from the hardware abstraction structure (drv_uart_shell_t)
 *
 * @warning This function may involve an hidden semaphore
 *
 * @param Pointer on h_shell_t structure.
 */
char sh_uartRead(h_shell_t* shell);

#endif /* INC_LIB_SHELL_SHELL_H_ */
