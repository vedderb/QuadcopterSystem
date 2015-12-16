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

#include "usartinterface.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/*
 * Private variables
 */
static DMA_InitTypeDef  DMA_InitStructure;
volatile static uint8_t txBuffer[USARTINTERFACE_TX_BUFFER_NUM][USARTINTERFACE_TX_BUFFER_SIZE];
volatile static unsigned int txLen[USARTINTERFACE_TX_BUFFER_NUM];
volatile static int txBusyFlag;
volatile static int txToSend[USARTINTERFACE_TX_BUFFER_NUM];
volatile static int lastTxBufferSent;

/*
 * Function pointers
 */
static void (*byteReceivedCallback)(uint8_t);

/*
 * Initialize the usartinterface. The used periperals are:
 * USART
 * DMA
 * Interrupts
 */
void usartinterface_init() {
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Initialize variables
	txBusyFlag = 0;
	lastTxBufferSent = 0;
	byteReceivedCallback = 0;

	int i;
	for(i = 0;i < USARTINTERFACE_TX_BUFFER_NUM;i++) {
		txLen[i] = 0;
		txToSend[i] = 0;
	}

	// Clock
	RCC_APB1PeriphClockCmd(USARTINTERFACE_RCC_USART, ENABLE);
	RCC_AHB1PeriphClockCmd(USARTINTERFACE_RCC_GPIO, ENABLE);
	RCC_AHB1PeriphClockCmd(USARTINTERFACE_RCC_DMA, ENABLE);

	// IO
	GPIO_InitStructure.GPIO_Pin = USARTINTERFACE_TXPIN | USARTINTERFACE_RXPIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USARTINTERFACE_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(USARTINTERFACE_GPIO, USARTINTERFACE_PINSRCTX, USARTINTERFACE_AFPIN);
	GPIO_PinAFConfig(USARTINTERFACE_GPIO, USARTINTERFACE_PINSRCRX, USARTINTERFACE_AFPIN);

	// UART Configuration
	USART_InitStructure.USART_BaudRate = USARTINTERFACE_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USARTINTERFACE_USART, &USART_InitStructure);

	// DMA Configuration
	DMA_InitStructure.DMA_PeripheralBaseAddr = USARTINTERFACE_DR_ADDRESS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Channel = USARTINTERFACE_DMA_USART_TX;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	// NVIC configuration
	// Configure the Priority Group to 2 bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USARTx Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USARTINTERFACE_USART_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the USARTx Receive interrupt: this interrupt is generated when the
	// USARTx receive data register is not empty
	USART_ITConfig(USARTINTERFACE_USART, USART_IT_RXNE, ENABLE);

	// Enable USART transmission complete interrupt.
	USART_ITConfig(USARTINTERFACE_USART, USART_IT_TC, ENABLE);

	// Enable USART
	USART_Cmd(USARTINTERFACE_USART, ENABLE);
}

/*
 * Set the function that should be called each time a byte is received on the USART.
 * This function should be called after usartinterface_init, as usartinterface_init
 * will clear the callback function pointer.
 */
void usartinterfaceSetByteReceivedCallback(void (*callbackFunction)(uint8_t)) {
	byteReceivedCallback = callbackFunction;
}

/*
 * Returns whether the tx buffer with bufferNum is free or not.
 */
int usartinterfaceIsTxBufferFree(unsigned int bufferNum) {
	if (bufferNum < USARTINTERFACE_TX_BUFFER_NUM) {
		return txToSend[bufferNum] == 0 ? 1 : 0;
	} else {
		return 0;
	}
}

/*
 * Add one byte to the end of the chosen tx buffer
 */
void usartinterfaceAddByteToTxBuffer(unsigned int bufferNum, uint8_t byte) {
	if (bufferNum < USARTINTERFACE_TX_BUFFER_NUM && txLen[bufferNum] < USARTINTERFACE_TX_BUFFER_SIZE) {
		txBuffer[bufferNum][txLen[bufferNum]++] = byte;
	}
}

/*
 * Add an array to the end of the chosen tx buffer. If the array does not fit
 * in the remaining buffer only the bytes that fit will be added.
 */
void usartinterfaceAddArrayToTxBuffer(unsigned int bufferNum, uint8_t *bytes, unsigned int len) {
	if (bufferNum < USARTINTERFACE_TX_BUFFER_NUM) {
		int i;
		for (i = 0;i < len;i++) {
			if (txLen[bufferNum] >= USARTINTERFACE_TX_BUFFER_SIZE) {
				break;
			}

			txBuffer[bufferNum][txLen[bufferNum]++] = bytes[i];
		}
	}
}

/*
 * Clear one tx buffer.
 */
void usartinterfaceClearTxBuffer(unsigned int bufferNum) {
	if (bufferNum < USARTINTERFACE_TX_BUFFER_NUM) {
		txLen[bufferNum] = 0;
	}
}

/*
 * Send one tx buffer.
 */
void usartinterfaceSendTxBuffer(unsigned int bufferNum) {
	if (bufferNum >= USARTINTERFACE_TX_BUFFER_NUM) {
		return;
	}

	txToSend[bufferNum] = 1;

	__disable_irq();
	if (txBusyFlag) {
		__enable_irq();
		return;
	}
	txBusyFlag = 1;
	__enable_irq();

	lastTxBufferSent = bufferNum;

	DMA_DeInit(USARTINTERFACE_DMA_STREAM_TX);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txBuffer[bufferNum];
	DMA_InitStructure.DMA_BufferSize = txLen[bufferNum];
	DMA_Init(USARTINTERFACE_DMA_STREAM_TX, &DMA_InitStructure);

	// Enable the USART DMA requests
	USART_DMACmd(USARTINTERFACE_USART, USART_DMAReq_Tx, ENABLE);

	// Enable the DMA TX Stream, USART will start sending
	DMA_Cmd(USARTINTERFACE_DMA_STREAM_TX, ENABLE);
}

/*
 * USART Interrupt handler. Handles reception of bytes. The end of DMA
 * transactions is also handled from here.
 */
void USARTINTERFACE_USART_IRQ() {
	if (USART_GetITStatus(USARTINTERFACE_USART, USART_IT_RXNE) != RESET) {
		/*
		 * Byte received on USART.
		 */

		// Always read byte to clear interrupt flag
		uint8_t recByte = USART_ReceiveData(USARTINTERFACE_USART) & 0x7F;

		// Use callback functions if it is not a null pointer
		if (byteReceivedCallback) {
			byteReceivedCallback(recByte);
		}
	} else if (USART_GetITStatus(USARTINTERFACE_USART, USART_IT_TC) != RESET) {
		/*
		 * Transfer complete flag set. If more buffers are to be sent send them
		 * using DMA, otherwise stop sending data.
		 */

		txToSend[lastTxBufferSent] = 0;

		// Clear the TC bit in the SR register by writing 0 to it
		USART_ClearFlag(USARTINTERFACE_USART, USART_FLAG_TC);

		// Check if any other buffer has pending data and send it if so
		int pendingData = 0;
		int i;
		for(i = 0;i < USARTINTERFACE_TX_BUFFER_NUM;i++) {
			if (txToSend[i]) {
				pendingData = 1;
				txBusyFlag = 1;
				lastTxBufferSent = i;

				DMA_DeInit(USARTINTERFACE_DMA_STREAM_TX);
				DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txBuffer[i];
				DMA_InitStructure.DMA_BufferSize = txLen[i];
				DMA_Init(USARTINTERFACE_DMA_STREAM_TX, &DMA_InitStructure);

				// Enable the USART DMA requests
				USART_DMACmd(USARTINTERFACE_USART, USART_DMAReq_Tx, ENABLE);

				// Enable the DMA TX Stream, USART will start sending
				DMA_Cmd(USARTINTERFACE_DMA_STREAM_TX, ENABLE);
				break;
			}
		}

		if (pendingData == 0) {
			txBusyFlag = 0;
		}
	}
}
