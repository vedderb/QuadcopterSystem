/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef USARTINTERFACE_H_
#define USARTINTERFACE_H_

#include <stdint.h>

/*
 * Public functions
 */
void usartinterface_init();
void usartinterfaceSetByteReceivedCallback(void (*callbackFunction)(uint8_t));
void usartinterfaceAddByteToTxBuffer(unsigned int bufferNum, uint8_t byte);
void usartinterfaceAddArrayToTxBuffer(unsigned int bufferNum, uint8_t *bytes, unsigned int len);
void usartinterfaceClearTxBuffer(unsigned int bufferNum);
void usartinterfaceSendTxBuffer(unsigned int bufferNum);
int usartinterfaceIsTxBufferFree(unsigned int bufferNum);

/*
 * Simple parameters
 */
#define USARTINTERFACE_BAUDRATE				9600	// Baudrate
#define USARTINTERFACE_TX_BUFFER_SIZE		256		// Size of each transmit buffer
#define USARTINTERFACE_TX_BUFFER_NUM		5		// Number of transmit buffers

/*
 * Advanced hardware-dependent parameters
 */
#define USARTINTERFACE_USART			UART4
#define USARTINTERFACE_RCC_USART		RCC_APB1Periph_UART4
#define USARTINTERFACE_RCC_GPIO			RCC_AHB1Periph_GPIOC
#define USARTINTERFACE_AFPIN			GPIO_AF_UART4
#define USARTINTERFACE_GPIO				GPIOC
#define USARTINTERFACE_TXPIN			GPIO_Pin_10
#define USARTINTERFACE_RXPIN			GPIO_Pin_11

#define USARTINTERFACE_PINSRCTX			GPIO_PinSource10
#define USARTINTERFACE_PINSRCRX			GPIO_PinSource11
#define USARTINTERFACE_USART_IRQ		UART4_IRQHandler
#define USARTINTERFACE_RCC_DMA			RCC_AHB1Periph_DMA1
#define USARTINTERFACE_DR_ADDRESS		((uint32_t)UART4 + 0x04)
#define USARTINTERFACE_DMA_USART_TX		DMA_Channel_4
#define USARTINTERFACE_DMA_STREAM_TX	DMA1_Stream4
#define USARTINTERFACE_USART_IRQN		UART4_IRQn


#endif /* USARTINTERFACE_H_ */
