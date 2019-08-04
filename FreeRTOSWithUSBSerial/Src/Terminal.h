/*
 * Terminal.h
 * idea from https://raw.githubusercontent.com/kylemanna/freertos/master/FreeRTOS/Demo/HCS12_CodeWarrior_banked/serial/serial.c
 *  Created on: 4 Aug 2019
 *      Author: chcao
 */
#ifdef ENABLE_ME
#ifndef TERMINAL_H_
#define TERMINAL_H_
#include "main.h"
#include <FreeRTOS.h>
#include <queue.h>

/**
 * The terminal queue length
 */
#define TERMINAL_QUEUE_LENGTH 256

class Terminal {
private:
	/**
	 * The queues used to communicate between the task and the interrupt service routines.
	 */
	QueueHandle_t rx_queue;
	QueueHandle_t tx_queue;
	UART_HandleTypeDef *huart;
public:
	Terminal();
	void initialise(UART_HandleTypeDef *huart);
	virtual ~Terminal();

	//functions can be called from a  normal task
	signed portBASE_TYPE GetChar(signed char *ret_char, TickType_t block_time);
	signed portBASE_TYPE PutChar(signed char out_char, TickType_t block_time);


	// Don't call this function from a normal task. It must be called from uart interupt!!
	void ISR(void);
};

#endif /* TERMINAL_H_ */
#endif // ENABLE_ME
