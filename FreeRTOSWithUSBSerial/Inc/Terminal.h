/*
 * Terminal.h
 * idea from https://raw.githubusercontent.com/kylemanna/freertos/master/FreeRTOS/Demo/HCS12_CodeWarrior_banked/serial/serial.c
 *  Created on: 4 Aug 2019
 *      Author: chcao
 */
#ifndef TERMINAL_H_
#define TERMINAL_H_
#include <FreeRTOS.h>
#include <queue.h>

/**
 * The terminal queue length
 */
#define TERMINAL_QUEUE_LENGTH 20

typedef struct
{
	QueueHandle_t rx_queue;
	QueueHandle_t tx_queue;
	UART_HandleTypeDef *huart;
} Terminal_t;
extern Terminal_t uart_terminal;

extern void term_initialise(Terminal_t *term, UART_HandleTypeDef *huart);
extern portBASE_TYPE term_get_char(Terminal_t *term, signed char *ret_char, TickType_t block_time);
portBASE_TYPE term_put_char(Terminal_t *term, signed char out_char, TickType_t block_time);

/**
 * UART calling from ISR
 */
extern void term_UART_ISR(Terminal_t *term);

#ifdef __cplusplus
 extern "C" {
#endif

void usb_receive_from_ISR(uint8_t* buffer, uint32_t count);
void usb_terminal_intialise();

void USBCommTask(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* TERMINAL_H_ */

