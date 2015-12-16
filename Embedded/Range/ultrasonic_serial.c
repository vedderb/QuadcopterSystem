/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se
	Copyright 2013-2014 Daniel Skarin	daniel.skarin@sp.se
	Copyright 2013-2014 Henrik Eriksson	henrik.eriksson@sp.se

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
#include <stdio.h>
#include <string.h>

#include "contiki-net.h"
#include "uip.h"
#include "udp_commands.h"
#include "buffer.h"
#include <math.h>

#include "ultrasonic_serial.h"
#include "usartinterface.h"
#include "packet.h"
#include "comm.h"

// Settings
#define MAX_WRONG_SAMPLES	3
#define SAMPLE_MAX_DIFF		0.5
#define UART4_BUFFER_SIZE	80

// Private variables
static char UART4_rx_buffer[UART4_BUFFER_SIZE];
static I2C_InitTypeDef I2C_InitStructure;

// Private functions
static void rxbyte_handler(uint8_t ch);
static void process_packet(unsigned char *buffer, unsigned char len);
static void send_packet(unsigned char *buffer, unsigned char len);
static int I2C_wait_event(I2C_TypeDef* I2Cx, uint32_t event);
static int I2C_wait_flag(I2C_TypeDef* I2Cx, uint32_t flag);
static void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
static int I2C_read_ack(I2C_TypeDef* I2Cx, uint8_t *data);
static int I2C_read_nack(I2C_TypeDef* I2Cx, uint8_t *data);
static void I2C_stop(I2C_TypeDef* I2Cx);

// Settings
#define I2C_EVENT_TIMEOUT_US	2000
#define I2C_FLAG_TIMEOUT_US		2000

#define STM32_CYCLES_PER_LOOP 6 // This will need tweaking or calculating
static void delay_us(uint32_t us) {
	us *= SystemCoreClock / 1000000 / STM32_CYCLES_PER_LOOP;

	asm volatile(" mov r0, %[us] \n\t"
			"1: subs r0, #1 \n\t"
			" bhi 1b \n\t"
			:
			: [us] "r" (us)
			: "r0");
}

static void SRF10_I2C_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable I2C and GPIO clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Microsecond delay timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);

	/* Configure I2C pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);

	/* Apply I2C configuration after enabling it */
	I2C_DeInit(I2C1);
	I2C_Init(I2C1, &I2C_InitStructure);
}

static int I2C_wait_event(I2C_TypeDef* I2Cx, uint32_t event) {
	TIM_Cmd(TIM13, ENABLE);
	TIM13->CNT = 0;

	while (!I2C_CheckEvent(I2Cx, event)) {
		if (TIM13->CNT > I2C_EVENT_TIMEOUT_US) {
			return 0;
		}
	}

	TIM_Cmd(TIM13, DISABLE);

	return 1;
}

static int I2C_wait_flag(I2C_TypeDef* I2Cx, uint32_t flag) {
	TIM13->CNT = 0;
	TIM_Cmd(TIM13, ENABLE);

	while (I2C_GetFlagStatus(I2Cx, flag)) {
		if (TIM13->CNT > I2C_FLAG_TIMEOUT_US) {
			return 0;
		}
	}

	TIM_Cmd(TIM13, DISABLE);

	return 1;
}


static void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
	// wait until I2C1 is not busy anymore
	if (!I2C_wait_flag(I2Cx, I2C_FLAG_BUSY )) {
		return;
	}

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
		return;
	}

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	if (direction == I2C_Direction_Transmitter ) {
		if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			return;
		}
	} else if (direction == I2C_Direction_Receiver ) {
		if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			return;
		}
	}
}

static void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		return;
	}
}

static int I2C_read_ack(I2C_TypeDef* I2Cx, uint8_t *data) {
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		return 0;
	}
	// read data from I2C data register and return data byte
	*data = I2C_ReceiveData(I2Cx);
	return 1;
}

static int I2C_read_nack(I2C_TypeDef* I2Cx, uint8_t *data) {
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	if (!I2C_wait_event(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		return 0;
	}
	// read data from I2C data register and return data byte
	*data = I2C_ReceiveData(I2Cx);
	return 1;
}

static void I2C_stop(I2C_TypeDef* I2Cx) {
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


static void SRF10_I2C_ByteWrite(u8 slaveAddr, u8 pBuffer, u8 writeAddr) {
	I2C_start(I2C1, slaveAddr, I2C_Direction_Transmitter);
	I2C_write(I2C1, writeAddr);
	I2C_write(I2C1, pBuffer);
	I2C_stop(I2C1);
}

static int SRF10_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead) {
	I2C_start(I2C1, slaveAddr, I2C_Direction_Transmitter);
	I2C_write(I2C1, readAddr);
	I2C_stop(I2C1);

	I2C_start(I2C1, slaveAddr, I2C_Direction_Receiver);

	// While there is data to be read
	while (NumByteToRead) {
		if (NumByteToRead == 1) {
			if (!I2C_read_nack(I2C1, pBuffer)) {
				return -1;
			}
			pBuffer++;
			NumByteToRead--;
		} else {
			if (!I2C_read_ack(I2C1, pBuffer)) {
				return -2;
			}
			pBuffer++;
			NumByteToRead--;
		}
	}

	I2C_stop(I2C1);

	return 0;
}

static void rxbyte_handler(uint8_t ch) {
	static int wrong_sample_cnt = 0;
	static uint8_t i = 0;
	static float ultrasonic_range = 0.0;

	if (i < (UART4_BUFFER_SIZE-1)) {
		UART4_rx_buffer[i++] = ch;
	}

	// Format for sensor: 'R' b3 b2 b1 b0 CR
	if (ch == '\r') {
		UART4_rx_buffer[i] = 0;
		i = 0;

		if (6 == (strlen(UART4_rx_buffer)) && (UART4_rx_buffer[0] == 'R') && (UART4_rx_buffer[5] == '\r')) {
			float range_tmp = (uint32_t)(UART4_rx_buffer[4] - '0');
			range_tmp += (uint32_t)(UART4_rx_buffer[3] - '0') * 10;
			range_tmp += ((uint32_t)(UART4_rx_buffer[2] - '0') * 100);
			range_tmp += ((uint32_t)(UART4_rx_buffer[1] - '0') * 1000);

			range_tmp /= 1000.0;

			if (fabsf(ultrasonic_range - range_tmp) < SAMPLE_MAX_DIFF ||
					wrong_sample_cnt >= MAX_WRONG_SAMPLES) {
				ultrasonic_range = range_tmp;
				wrong_sample_cnt = 0;

				int32_t i = 0;
				uint8_t buf[20];
				buffer_append_int16(buf, ultrasonic_range * 1000.0, &i);
				comm_sendmsg(PACKET_INT_CMD_ALTITUDE, buf, i);

				leds_toggle(LEDS_RED);
			} else {
				wrong_sample_cnt++;
			}
		}
	}
}

static void process_packet(unsigned char *buffer, unsigned char len) {
	(void) buffer;
	(void) len;
}

static void send_packet(unsigned char *buffer, unsigned char len) {
	int i;
	for (i = 0;i < USARTINTERFACE_TX_BUFFER_NUM;i++) {
		if (usartinterfaceIsTxBufferFree(i)) {
			usartinterfaceClearTxBuffer(i);
			usartinterfaceAddArrayToTxBuffer(i, buffer, len);
			usartinterfaceSendTxBuffer(i);
			break;
		}
	}
}

static uint16_t SRF10_GetRange() {
	uint16_t tmp = 0;
	unsigned char rx_buf[2];

	//Read measurement
	rx_buf[0] = 0;
	rx_buf[1] = 0;
	SRF10_I2C_BufferRead(SRF10_DEFAULT_ADDRESS, rx_buf, SRF10_RANGE_REG, 2);

	tmp = rx_buf[0] << 8 | rx_buf[1];

	return tmp;
}

static void SRF10_SetGain(uint8_t gain) {
	SRF10_I2C_ByteWrite(SRF10_DEFAULT_ADDRESS, gain, SRF10_GAIN_REG);
}

/**
 * Distance = range * 43mm + 43mm
 */
static void SRF10_SetRange(uint8_t range) {
	SRF10_I2C_ByteWrite(SRF10_DEFAULT_ADDRESS, range, SRF10_RANGE_REG);
}

void ultrasonic_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	//I2C for SRF10 init
	SRF10_I2C_Init();

	//USART init
	usartinterface_init();
	usartinterfaceSetByteReceivedCallback(rxbyte_handler);

	// Configure PC12 as output to trigger "real-time" readings
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*
	 * TODO: Remember that sampling has to be triggered with GPIO C12. This
	 * is currently done in range.c
	 */

	packet_init(send_packet, process_packet, 0);
}

void srf10_start_range_handler() {
	//Set SRF10 parameters
	SRF10_SetGain(5);
	delay_us(100);
	SRF10_SetRange(92);
	delay_us(100);

	//Start measurement
	SRF10_I2C_ByteWrite(SRF10_DEFAULT_ADDRESS, 0x51, SRF10_MEAS_REG);
}

void srf10_get_range_handler() {
	float range_tmp = (float)SRF10_GetRange() / 100.0;
	static float ultrasonic_range = 0.0;
	static int wrong_sample_cnt = 0;

	if (fabsf(ultrasonic_range - range_tmp) < SAMPLE_MAX_DIFF ||
			wrong_sample_cnt >= MAX_WRONG_SAMPLES) {
		ultrasonic_range = range_tmp;
		wrong_sample_cnt = 0;

		int32_t i = 0;
		uint8_t buf[20];
		buffer_append_int16(buf, ultrasonic_range * 1000.0, &i);
		comm_sendmsg(PACKET_INT_CMD_ALTITUDE, buf, i);

		leds_toggle(LEDS_RED);
	} else {
		wrong_sample_cnt++;
	}
}
