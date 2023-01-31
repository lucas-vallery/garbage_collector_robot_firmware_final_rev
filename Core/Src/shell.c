/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"

static char g_backspaceString[] = "\b \b";
static char g_promptString[] = "> ";
char g_statistics[256];

static int sh_uartWrite(h_shell_t* shell, char * s, uint16_t size) {
	shell->uart.transmit(s, size);
	return size;
}

int sh_stats(h_shell_t* shell, int argc, char ** argv){
	vTaskGetRunTimeStats((char*) &g_statistics);
	printf("%s", g_statistics);
	return 0;
}

static int sh_help(h_shell_t* shell, int argc, char ** argv) {
	int i;
	for(i = 0 ; i < shell->shell_func_list_size ; i++) {
		int size;
		size = snprintf (shell->print_buffer, BUFFER_SIZE, "%s:\t\t%s\r\n", shell->shell_func_list[i].c, shell->shell_func_list[i].description);
		sh_uartWrite(shell, shell->print_buffer, size);
	}
	return 0;
}

static void shellTask (void * pvParameters){
	h_shell_t* shell = (h_shell_t*) pvParameters;

	sh_run(shell);
}

static int sh_exec(h_shell_t* shell, char * buf) {
	int i;

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	argc = 1;
	argv[0] = buf;
	for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
		if(*p == ' ' || *p == '\n') {
			*p = '\0';
			argv[argc] = p+1;
			argc++;
		}
	}

	for(i = 0 ; i < shell->shell_func_list_size ; i++) {
		if (strcmp(shell->shell_func_list[i].c, argv[0]) == 0)
			return shell->shell_func_list[i].func(shell, argc, argv);
	}

	int size;
	size = snprintf (shell->print_buffer, BUFFER_SIZE, "%s: no such command\r\n", argv[0]);
	sh_uartWrite(shell, shell->print_buffer, size);
	return -1;
}

char sh_uartRead(h_shell_t* shell) {
	char c;

	shell->uart.receive(&c, 1);

	return c;
}

void sh_init(h_shell_t* shell) {
	int size = 0;
	shell->shell_func_list_size = 0;
	shell->shellTaskHandler = NULL;


	size = snprintf (shell->print_buffer, BUFFER_SIZE, "\r\n\r\n===== Bière Dynamics Debug Tool =====\r\n");
	sh_uartWrite(shell, shell->print_buffer, size);

	sh_add(shell, "help", sh_help, "Help");
	sh_add(shell, "stats", sh_stats, "CPU occupation of the running tasks");
}


void sh_start(h_shell_t* shell) {
	if(pdTRUE != xTaskCreate(shellTask, "shellTask", STACK_DEPTH, (void*) shell, SHELL_TASK_PRIORITY, &(shell->shellTaskHandler))) {
		printf("Shell task failed");
		while(1);
	}
}

int sh_add(h_shell_t* shell, char* c, int (* pfunc)(h_shell_t* shell, int argc, char ** argv), char * description) {
	if (shell->shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		shell->shell_func_list[shell->shell_func_list_size].c = c;
		shell->shell_func_list[shell->shell_func_list_size].func = pfunc;
		shell->shell_func_list[shell->shell_func_list_size].description = description;
		shell->shell_func_list_size++;
		return 0;
	}
	return -1;
}

int sh_run(h_shell_t* shell) {
	int reading = 0;
	int pos = 0;

	static char cmd_buffer[BUFFER_SIZE];

	while(1) {
		sh_uartWrite(shell, g_promptString, 2);
		reading = 1;

		while(reading) {
			//printf("shell reading loop in\r\n");
			char c = sh_uartRead(shell);
			int size;

			switch(c){
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (shell->print_buffer, BUFFER_SIZE, "\r\n");
				sh_uartWrite(shell, shell->print_buffer, size);
				cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (shell->print_buffer, BUFFER_SIZE, ":%s\r\n", cmd_buffer);
				sh_uartWrite(shell, shell->print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					sh_uartWrite(shell, g_backspaceString, 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					sh_uartWrite(shell, &c, 1);
					cmd_buffer[pos++] = c; //store
				}
			}
		}
		sh_exec(shell, cmd_buffer);
	}
	return 0;
}

//The two following functions need to be defined to get the task statistics
void configureTimerForRunTimeStats(void){

}
unsigned long getRunTimeCounterValue(void){
	//return 0;
}
