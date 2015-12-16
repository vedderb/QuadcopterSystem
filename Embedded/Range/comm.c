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

#include "contiki.h"
#include "stm32f4xx_conf.h"
#include "dev/leds.h"
#include "comm.h"
#include "packet.h"

// Private variables
static CanTxMsg TxMessage;

void comm_init(void) {
#if COMM_USE_CAN
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	// Interrupts
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Connect CAN pins */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* CAN configuration ********************************************************/
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* CAN Baudrate = 125kbps (CAN clocked at 42 MHz) */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_Prescaler = 4;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Transmit Structure preparation */
	TxMessage.StdId = 0x321;
	TxMessage.ExtId = 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
#endif
}

void CAN1_RX0_IRQHandler(void) {
	CanRxMsg can_rx_msg;
	CAN_Receive(CAN1, CAN_FIFO0, &can_rx_msg);

	// TODO: Process message
}

void comm_sendmsg(uint8_t id, uint8_t *data, uint8_t len) {
#if COMM_USE_CAN
	if (len > 8) {
		len = 8;
	}

	TxMessage.StdId = id;
	TxMessage.DLC = len;

	int i;
	for (i = 0;i < len;i++) {
		TxMessage.Data[i] = data[i];
	}

	CAN_Transmit(CAN1, &TxMessage);
#else
	uint8_t buf[len + 1];

	buf[0] = PACKET_INT_CMD_CLOCK;

	int i = 0;
	for (i = 0;i < len;i++) {
		buf[i + 1] = data[i];
	}

	packet_send_packet(buf, len + 1, 0);
#endif
}

/**
 * If a transmission is ongoing, the next message can will be delayed until that transmission
 * is done.
 */
int comm_is_tx_busy(void) {
	int ok = 1;

#if COMM_USE_CAN
	if (CAN_TransmitStatus(CAN1, 0) == CAN_TxStatus_Pending ||
			CAN_TransmitStatus(CAN1, 1) == CAN_TxStatus_Pending ||
			CAN_TransmitStatus(CAN1, 2) == CAN_TxStatus_Pending) {
		ok = 0;
	}
#else
	int i;
	for (i = 0;i < USARTINTERFACE_TX_BUFFER_NUM;i++) {
		if (!usartinterfaceIsTxBufferFree(i)) {
			ok = 0;
			break;
		}
	}
#endif

	return !ok;
}
