#if 0

/**
 * TODO attempt to write ISR
 * - circular buffer to sending - queue!
 * - buffer to receive data
 *
 *  Copy from https://raw.githubusercontent.com/kylemanna/freertos/master/FreeRTOS/Demo/HCS12_CodeWarrior_banked/serial/serial.c
 */
#include "main.h"
#include <FreeRTOS.h>
#include <queue.h>


/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER for port 1.

Note that this driver is written to test the RTOS port and is not intended
to represent an optimised solution. */

/* Processor Expert generated includes. */
#include "com0.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application include files. */
#include "serial.h"

/* The queues used to communicate between the task code and the interrupt
service routines. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

/* Interrupt identification bits. */
#define serOVERRUN_INTERRUPT		( 0x08 )
#define serRX_INTERRUPT				( 0x20 )
#define serTX_INTERRUPT				( 0x80 )

-------------------------------------------------------*/


/*
 * Interrupt service routine for the serial port.  Must be in non-banked
 * memory.
 */

#pragma CODE_SEG __NEAR_SEG NON_BANKED

void vCOM0_ISR( void )
{
volatile unsigned char ucByte, ucStatus;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* What caused the interrupt? */
	ucStatus = SCI0SR1;

	if( ucStatus & serOVERRUN_INTERRUPT )
	{
		/* The interrupt was caused by an overrun.  Clear the error by reading
		the data register. */
		ucByte = SCI0DRL;
	}

	if( ucStatus & serRX_INTERRUPT )
	{
		/* The interrupt was caused by a character being received.
		Read the received byte. */
		ucByte = SCI0DRL;

		/* Post the character onto the queue of received characters - noting
		whether or not this wakes a task. */
		xQueueSendFromISR( xRxedChars, ( void * ) &ucByte, &xHigherPriorityTaskWoken );
	}

	if( ( ucStatus & serTX_INTERRUPT ) && ( SCI0CR2_SCTIE ) )
	{
		/* The interrupt was caused by a character being transmitted. */
		if( xQueueReceiveFromISR( xCharsForTx, ( void * ) &ucByte, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* Clear the SCRF bit. */
			SCI0DRL = ucByte;
		}
		else
		{
			/* Disable transmit interrupt */
			SCI0CR2_SCTIE = 0;
		}
	}

	if( xHigherPriorityTaskWoken )
	{
		portYIELD();
	}
}

#pragma CODE_SEG DEFAULT

#endif
