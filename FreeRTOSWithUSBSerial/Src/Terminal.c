/*
 * Terminal.cpp
 *  https://community.st.com/s/question/0D50X00009XkgrbSAB/best-way-to-use-hal-uart-receiver-it-function
 *	https://stackoverflow.com/questions/37336527/why-does-uart-transmit-interrupt-fail-to-work-in-this-case?noredirect=1&lq=1
 *	https://community.st.com/s/question/0D50X00009XkfrWSAR/cubemx-uart-receive-complete-interrupt
 *
 *  Created on: 4 Aug 2019
 *      Author: chcao
 */

#include "main.h"


Terminal_t term1;

void
term_initialise(Terminal_t *term, UART_HandleTypeDef *huart)
{
	term->rx_queue = xQueueCreate(TERMINAL_QUEUE_LENGTH, (unsigned portBASE_TYPE)sizeof(signed char));
	term->tx_queue = xQueueCreate(TERMINAL_QUEUE_LENGTH, (unsigned portBASE_TYPE)sizeof(signed char));
	term->huart = huart;
}

portBASE_TYPE
term_get_char(Terminal_t *term, signed char *ret_char, TickType_t block_time)
{
	if (xQueueReceive(term->rx_queue, ret_char, block_time))
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

portBASE_TYPE
term_put_char(Terminal_t *term, signed char out_char, TickType_t block_time)
{
	if (xQueueSend(term->tx_queue, &out_char, block_time) != pdPASS)
	{
		return pdFAIL;
	}

	//@todo
	// Turn on the Tx interrupt so the ISR will remove the character from the queue and send it.

	return pdPASS;
}

static void read_one_byte(Terminal_t *term)
{
	UART_HandleTypeDef *huart = term->huart;
	volatile unsigned char ucByte;
	uint16_t uhMask = huart->Mask;
	uint16_t uhdata;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;


	/* Check that a Rx process is ongoing */
	if (huart->RxState == HAL_UART_STATE_BUSY_RX)
	{
		uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
		ucByte = (uint8_t) (uhdata & (uint8_t) uhMask);

		/*
		 * Post the character onto the queue of received characters - noting
		 * whether or not this wakes a task.
		 */
		if (xQueueSendFromISR(term->tx_queue, (void * )&ucByte, &xHigherPriorityTaskWoken) != pdTRUE)
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupts */
			CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

			/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

			/* Rx process is completed, restore huart->RxState to Ready */
			huart->RxState = HAL_UART_STATE_READY;
		}
		else
		{
			if (xHigherPriorityTaskWoken)
			{
				//taskYIELD_YIELD_FROM_ISR();
			}
		}
	}	//busy
	else
	{
		/* Clear RXNE interrupt flag */
		__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	}
}

static void write_one_byte(Terminal_t *term)
{
	UART_HandleTypeDef *huart = term->huart;
	volatile unsigned char ucByte;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (huart->gState == HAL_UART_STATE_BUSY_TX)
	{
		if (xQueueReceiveFromISR(term->tx_queue, (void *)&ucByte, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			huart->Instance->TDR = (uint8_t)(ucByte & (uint8_t)0xFF);
		}
		else
		{
			/* Disable the UART Transmit Data Register Empty Interrupt */
			CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);

			/* Enable the UART Transmit Complete Interrupt */
			SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
		}
	}
}


/**
 * Interrupt service routine  - copy mostly from HAL layer
 */
void term_ISR(Terminal_t *term)
{
	UART_HandleTypeDef *huart = term->huart;
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);

  uint32_t errorflags;
  uint32_t errorcode;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == 0U)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if (((isrflags & USART_ISR_RXNE) != 0U)
        && ((cr1its & USART_CR1_RXNEIE) != 0U))
    {
      read_one_byte(term);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != 0U)
      && (((cr3its & USART_CR3_EIE) != 0U)
          || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != 0U)))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);

      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);

      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_ORE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE) != 0U) ||
            ((cr3its & USART_CR3_EIE) != 0U)))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver ---------------------------------------------------*/
      if (((isrflags & USART_ISR_RXNE) != 0U)
          && ((cr1its & USART_CR1_RXNEIE) != 0U))
      {
        read_one_byte(term);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      errorcode = huart->ErrorCode;
      if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) ||
          ((errorcode & HAL_UART_ERROR_ORE) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        //UART_EndRxTransfer(huart);

      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
        huart->ErrorCode = HAL_UART_ERROR_NONE;
      }
    }
    return;

  } /* End if some error occurs */

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_ISR_TXE) != 0U)
      && ((cr1its & USART_CR1_TXEIE) != 0U))
  {
    write_one_byte(term);
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U))
  {
    //UART_EndTransmit_IT(huart);
    return;
  }

}
