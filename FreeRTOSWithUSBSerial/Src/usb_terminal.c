/*
 * usb_terminal.cpp
 *
 *  Created on: 11 Aug 2019
 *      Author: chcao
 */
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <task.h>

#define USB_TERMINAL_QUEUE_LENGTH (100)

QueueHandle_t usb_rx_queue;

void usb_terminal_intialise()
{
	usb_rx_queue = xQueueCreate(USB_TERMINAL_QUEUE_LENGTH, (unsigned portBASE_TYPE)sizeof(signed char));
}


// calling from ISR CDC_Receive_HS()
void usb_receive_from_ISR(uint8_t* buffer, uint32_t count)
{
	// We have not woken a task at the start of the ISR.
	portBASE_TYPE xHigherPriorityTaskWokenByPost = pdFALSE;

	for (uint32_t i = 0; i < count; i++)
		xQueueGenericSendFromISR(usb_rx_queue, &buffer[i], &xHigherPriorityTaskWokenByPost, queueSEND_TO_BACK);

	// Now the buffer is empty we can switch context if necessary.  Note that the
	// name of the yield function required is port specific.
	if (xHigherPriorityTaskWokenByPost)
		taskYIELD(); //http://www.freertos.org/a00120.html
}

/**
  * @brief  Function implementing the USB communication task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void USBCommTask(void const * argument)
{
	  int count = 0;
	  char *message = "USB Virtual COM port\r\n";
	  unsigned char input;
	  unsigned char ret[2] = {'\n', '\r'};

	  HAL_GPIO_WritePin(GPIOJ, LD_USER1_Pin, GPIO_PIN_SET);
	  osDelay(2000);
	  CDC_Transmit_HS((uint8_t*)message, (uint16_t)strlen(message));
	  osDelay(100);
	  HAL_GPIO_WritePin(GPIOJ, LD_USER1_Pin, GPIO_PIN_RESET);
	  for(;;) 	  /* Infinite loop */
	  {
		  count++;

		  //CDC_Transmit_HS((uint8_t*)message, (uint16_t)strlen(message));
		  //osDelay(2000);
		  if (xQueueReceive(usb_rx_queue, &input, portMAX_DELAY))
		  {
			  if (input == '\r')
				  CDC_Transmit_HS((uint8_t*)ret, (uint16_t)2);
			  else
				  CDC_Transmit_HS((uint8_t*)&input, (uint16_t)1);
			  count++;
			  if (count & 1)
				  HAL_GPIO_WritePin(GPIOJ, LD_USER1_Pin, GPIO_PIN_SET);
			  else
				  HAL_GPIO_WritePin(GPIOJ, LD_USER1_Pin, GPIO_PIN_RESET);

		  }
		  taskYIELD();
	  }
}




