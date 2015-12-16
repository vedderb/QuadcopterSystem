/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se
	Copyright 2013-2014 Daniel Skarin	daniel.skarin@sp.se

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

#include "stm32f4xx_conf.h"
#include "signal.h"
#include "range.h"
#include "dev/leds.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <arm_math.h>
#include <arm_const_structs.h>

#include "udp_commands.h"

#include "packet.h"
#include "buffer.h"
#include "ultrasonic_serial.h"
#include "comm.h"

// Pulse
#define PULSE_LENGTH			800 //200
#define SAMP_SPEED				200000
#define F_ULTRA					40000
#define TIM_PERIOD				(((SystemCoreClock / 2 / SAMP_SPEED) - 1))

// FFT Settings
#define FFT_N					2048
#define FFT_M					PULSE_LENGTH
#define FFT_L					(FFT_N - FFT_M + 1)
#define FFT_OVERLAP				(FFT_M - 1)

// ADC
#define ADC1_DR_ADDRESS			((uint32_t)0x4001204C)
#define ADC_BUFFER_LENGTH		9000
#define CORRELATION_DATA_LENGTH	ADC_BUFFER_LENGTH

// Correlation analysis
#define CORR_TRES				(1200.0 / 2000.0)
#define CORR_TRES_DIFF			250
#define CORR_START_SKIP			20

// Misc
#define SPEED_OF_SOUND			340.2
//#define TRANSMISSION_DELAY		1110
#define TRANSMISSION_DELAY		50
#define SEND_LEN				500 // The number of samples to send to plot

// Macros
#define MAX(i1, i2) (i1 > i2 ? i1 : i2)

// Private functions
static float index_to_distance(uint32_t sample);
inline static uint32_t get_tres_index(uint32_t index, float32_t last_max,
		uint32_t last_index, float32_t *tres);
inline static float32_t update_max(float32_t sample_now,
		float32_t *prev_samples, uint32_t *prev_sample_ind);

static uint32_t xcorr_rf32(uint16_t* data, int buffer_length, uint16_t* correlation_data);
static void xcorr_init_rf32(uint16_t* pulse, uint16_t pulse_length);

// Private variables
static uint16_t adc_buffer[2][ADC_BUFFER_LENGTH];
static uint16_t pulse_buffer[PULSE_LENGTH];
static uint16_t correlation_buffer[CORRELATION_DATA_LENGTH];
static struct uip_udp_conn *udp_conn = 0;
static MEASUREMENT_DATA measurements[ANCHOR_NUM];

// FFT
static arm_rfft_fast_instance_f32 rfft_instance;
static float32_t fft_t[FFT_N];
static float32_t pulse_f[FFT_N];
static float32_t axb_f[FFT_N];

static uint8_t print_map_udp = 0;

PROCESS(corr_process, "Correlation process");

PROCESS_THREAD(corr_process, ev, data) {
	PROCESS_BEGIN();

	xcorr_init_rf32(pulse_buffer, PULSE_LENGTH);

	for(;;) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

//		signal_compute(0, 0);
	}

	PROCESS_END();
	return 0;
}

void signal_compute(uint32_t timeslot, int adc_buffer_num, CMD_CLIENT_t cmd) {
	unsigned char buf[SEND_LEN + 1];
	static uint16_t tmp_adc_buffer[ADC_BUFFER_LENGTH];

	uint32_t samp_avg = 0;
	uint32_t pulse_position = 0;
	int32_t i = 0;

	leds_on(LEDS_GREEN);

	memcpy(tmp_adc_buffer, adc_buffer[adc_buffer_num], sizeof(tmp_adc_buffer));

	// Locate symbol using cross-correlation
	pulse_position = xcorr_rf32(tmp_adc_buffer, ADC_BUFFER_LENGTH, correlation_buffer);

	uip_ip6addr(&udp_conn->ripaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
	udp_conn->rport = UIP_HTONS(3001);

	if (cmd == CMD_CLIENT_SEND_PULSE_SAMPLE) {
		// Compute the average value of the sampled data
		samp_avg = 0;
		for (i = 0;i < ADC_BUFFER_LENGTH;i++) {
			samp_avg += tmp_adc_buffer[i];
		}
		samp_avg /= ADC_BUFFER_LENGTH;

		// Remove the average - 128 of each sample (to get the variation
		// on the LSBs) and put it in the send buffer
		for(i = 0;i < SEND_LEN;i++) {
			unsigned int samp = tmp_adc_buffer[i + pulse_position];
			samp -= (samp_avg - 128);
			buf[i + 1] = samp;
		}
		buf[0] = CMD_SERVER_LOGGED_PULSE;
		uip_udp_packet_send(udp_conn, buf, SEND_LEN + 1);

		// Send correlation data
		for(i = 0;i < (SEND_LEN / 2);i++) {
			unsigned int samp = correlation_buffer[i + (pulse_position - 125)];
			buf[2 * i + 1] = (samp >> 8) & 0xFF;
			buf[2 * i + 2] = samp & 0xFF;
		}
		buf[0] = CMD_SERVER_LOGGED_CORRELATION;
		uip_udp_packet_send(udp_conn, buf, SEND_LEN + 1);
	}

	float distance = index_to_distance(pulse_position);
	measurements[timeslot].distance = distance;
	i = 0;
	buf[i++] = timeslot;
	buffer_append_int32(buf, distance * 1000.0, &i);
	comm_sendmsg(PACKET_INT_CMD_RANGE, buf, i);

	if (print_map_udp) {
		// Print the measured value
		buf[0] = CMD_SERVER_TEXT;
		sprintf((char *)buf+1, "Index: %u, distance: %.3f Slot: %lu",
				(unsigned int)pulse_position, (double)distance, timeslot);
		uip_udp_packet_send(udp_conn, (char *) buf, strlen((char *) buf+1) + 1);

		// Send all measurements
		for(i = 0;i < ANCHOR_NUM;i++) {
			buf[0] = CMD_SERVER_DISTANCES;
			int32_t dist = (int32_t)(measurements[i].distance * 1000.0);

			buf[4 * i + 1] = dist >> 24;
			buf[4 * i + 2] = dist >> 16;
			buf[4 * i + 3] = dist >> 8;
			buf[4 * i + 4] = dist;
		}
		uip_udp_packet_send(udp_conn, (char *) buf, ANCHOR_NUM * 4 + 1);
	}

	if (cmd == CMD_CLIENT_SEND_PULSE_SAMPLE_ALLDATA) {
		// Send all logged data
		buf[0] = CMD_SERVER_LOGGED_PULSE_PART;
		buf[1] = 0; // Pad for byte alignment
		uint16_t *buf_u16 = (uint16_t *) (buf+2);
		buf_u16[0] = 0;
		int j = 1; // u16 index

		for (i = 0; i < ADC_BUFFER_LENGTH; i++) {
			buf_u16[j++] = uip_htons(tmp_adc_buffer[i]);

			if (j == (SEND_LEN-2)/2) {
				uip_udp_packet_send(udp_conn, buf, SEND_LEN);

				buf[0] = CMD_SERVER_LOGGED_PULSE_PART;
				buf_u16[0] = uip_htons(i);
				j = 1;
			}
		}

		buf[0] = CMD_SERVER_LOGGED_CORRELATION_PART;
		buf[1] = 0; // Pad for byte alignment
		buf_u16 = (uint16_t *) (buf+2);
		buf_u16[0] = 0;
		j = 1; // u16 index

		for (i = 0; i < (CORRELATION_DATA_LENGTH); i++) {
			buf_u16[j++] = uip_htons(correlation_buffer[i]);

			if (j == (SEND_LEN-2)/2) {
				uip_udp_packet_send(udp_conn, buf, SEND_LEN);

				buf[0] = CMD_SERVER_LOGGED_CORRELATION_PART;
				buf_u16[0] = uip_htons(i);
				j = 1;
			}
		}
	}

	// Restore server connection to allow data from any node
	memset(&udp_conn->ripaddr, 0, sizeof(udp_conn->ripaddr));
	leds_off(LEDS_GREEN);
}

void signal_init(void) {
	static uip_ipaddr_t server_addr;
	uip_ip6addr(&server_addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
	udp_conn = udp_new(&server_addr, UIP_HTONS(3001), NULL );

	int i;
	for (i = 0;i < PULSE_LENGTH;i++) {
		float sinc_tmp = (float)(i - (PULSE_LENGTH / 2)) * M_PI * 2.0 / (float)(PULSE_LENGTH / 2);
		float x = (float)(i - (PULSE_LENGTH / 2)) * M_PI * (float)PULSE_LENGTH * ((float)F_ULTRA / (float)SAMP_SPEED) / (float)(PULSE_LENGTH / 2);

		float y = 1;
		if (sinc_tmp != 0) {
			y = sinf(sinc_tmp)  / sinc_tmp;
		}
		// Scale signal to [0, TIM_PERIOD]
		pulse_buffer[i] =  (int)(((y * sinf(x)) + 1.0f) * (float)TIM_PERIOD * 0.5f);
	}
}


static void xcorr_init_rf32(uint16_t* pulse, uint16_t pulse_length) {
	uint32_t i = 0;

	// fft and output in normal order (bitReverseFlag = 1)
	arm_status status = arm_rfft_fast_init_f32(&rfft_instance, FFT_N);
	if (status != ARM_MATH_SUCCESS) {
		while(1);
	}

	arm_fill_f32(0.0, fft_t, FFT_N);
	// Scale to +/- 1
	for (i = 0; i < pulse_length; i++) {
		fft_t[i] = ((float32_t) pulse[i] / (TIM_PERIOD * 0.5f)) - 1.0f;
	}
	arm_rfft_fast_f32(&rfft_instance, fft_t, pulse_f, 0);
}

static uint32_t xcorr_rf32(uint16_t* data, int buffer_length, uint16_t* correlation_data){
	uint32_t i = 0;
	uint32_t pos = 0;
	uint32_t max_i = 0;
	float32_t max_value = 0.0;
	uint32_t tres_max_index = 0;
	float32_t tres = 0.0;


#define PREV_SAMPLE_LEN		5
	float32_t m0 = 0.0f, m1 = 0.0f, m2 = 0.0f;
	float32_t tmax;
	float32_t  prev_samples[PREV_SAMPLE_LEN];
	uint32_t prev_sample_ind = 0;
	arm_fill_f32(0.0f, prev_samples, PREV_SAMPLE_LEN);

	while((pos + FFT_L) <= buffer_length) {
		if (pos == 0) {
			// First block: add M-1 zeros in the beginning
			arm_fill_f32(0.0f, fft_t, FFT_M-1);
			for (i = 0; i < FFT_L; i++) {
				fft_t[i + FFT_M - 1] = (float32_t) data[i] / 4096.0f ;
			}
		} else {
			// Input block n: copy M-1 bytes of previous block and L bytes of current block
			for (i = 0; i < FFT_N; i++) {
				fft_t[i] = (float32_t) data[pos + i - (FFT_M - 1)] / 4096.0f ;
			}
		}

		arm_rfft_fast_f32(&rfft_instance, fft_t, axb_f, 0);
		arm_cmplx_mult_cmplx_f32(pulse_f, axb_f, axb_f, FFT_N/2);
		arm_rfft_fast_f32(&rfft_instance, axb_f, fft_t, 1);

		uint32_t start = (FFT_M-1);
		if (pos == 0) {
			start *= 2;
		}

		for (i = start; i < FFT_N; i++) {
			const int32_t corr_index = pos + i - 2 * (FFT_M-1);
			const float32_t sample_now = fft_t[i];

			if (corr_index == 0) {
				arm_fill_f32(sample_now, prev_samples, PREV_SAMPLE_LEN);
			}

			m0 = update_max(sample_now, prev_samples, &prev_sample_ind);
			tmax = MAX(MAX(m0, m1), m2);
			m2 = m1;
			m1 = m0;

			correlation_data[corr_index] = (uint16_t) (tmax * 2000.0);

			if (!tres_max_index) {
				if (corr_index > CORR_START_SKIP && tmax > max_value) {
					max_i = corr_index;
					max_value = tmax;
				}

				tres_max_index = get_tres_index(corr_index, max_value, max_i, &tres);
			}

//			if (tmax > max_value) {
//				max_value = tmax;
//				tres_max_index = corr_index;
//			}
		}

		pos += FFT_L;
	}

	return tres_max_index;
}

inline static uint32_t get_tres_index(uint32_t index, float32_t last_max,
		uint32_t last_index, float32_t *tres) {
	if (index < TRANSMISSION_DELAY) {
		*tres = last_max + CORR_TRES;
		return 0;
	}

	if ((index - last_index) > CORR_TRES_DIFF && last_max > *tres) {
		return last_index;
	} else {
		return 0;
	}
}

inline static float32_t update_max(float32_t sample_now,
		float32_t *prev_samples, uint32_t *prev_sample_ind) {
	float32_t t1, t2, t3, t4;

	switch (*prev_sample_ind) {
	case 0:
		t1 = fabsf(sample_now - prev_samples[3]);
		t2 = fabsf(sample_now - prev_samples[2]);
		t3 = fabsf(sample_now - prev_samples[1]);
		t4 = fabsf(sample_now - prev_samples[0]);
		prev_samples[(*prev_sample_ind)++] = sample_now;
		break;

	case 1:
		t1 = fabsf(sample_now - prev_samples[4]);
		t2 = fabsf(sample_now - prev_samples[3]);
		t3 = fabsf(sample_now - prev_samples[2]);
		t4 = fabsf(sample_now - prev_samples[1]);
		prev_samples[(*prev_sample_ind)++] = sample_now;
		break;

	case 2:
		t1 = fabsf(sample_now - prev_samples[0]);
		t2 = fabsf(sample_now - prev_samples[4]);
		t3 = fabsf(sample_now - prev_samples[3]);
		t4 = fabsf(sample_now - prev_samples[2]);
		prev_samples[(*prev_sample_ind)++] = sample_now;
		break;

	case 3:
		t1 = fabsf(sample_now - prev_samples[1]);
		t2 = fabsf(sample_now - prev_samples[0]);
		t3 = fabsf(sample_now - prev_samples[4]);
		t4 = fabsf(sample_now - prev_samples[3]);
		prev_samples[(*prev_sample_ind)++] = sample_now;
		break;

	case 4:
		t1 = fabsf(sample_now - prev_samples[2]);
		t2 = fabsf(sample_now - prev_samples[1]);
		t3 = fabsf(sample_now - prev_samples[0]);
		t4 = fabsf(sample_now - prev_samples[4]);
		prev_samples[*prev_sample_ind] = sample_now;
		*prev_sample_ind = 0;
		break;

	default:
		t1 = 0;
		t2 = 0;
		t3 = 0;
		t4 = 0;
		break;
	}

	return MAX(MAX(t1, t2), MAX(t3, t4));
}

inline static float index_to_distance(uint32_t sample) {
	if (sample <= TRANSMISSION_DELAY) {
		return 0.0;
	}

	return ((((float)sample - (float)TRANSMISSION_DELAY) / (float)SAMP_SPEED))
			* (float)SPEED_OF_SOUND;
}

void signal_generate_pulse(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// GPIOB clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// GPIOB Configuration: Channel 3 as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

	// DMA clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);

	DMA_DeInit(DMA1_Stream7);
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR3;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pulse_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = PULSE_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream7, &DMA_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// Channel 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pulse_buffer[0];
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	// TIM4 counter enable
	TIM_Cmd(TIM4, ENABLE);

	// DMA enable
	DMA_Cmd(DMA1_Stream7, ENABLE);

	// TIM4 Update DMA Request enable
	TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM4, ENABLE);

	// Enable DMA transfer complete interrupt
	DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Stream7_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7)) {
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7 );

		TIM_DeInit(TIM4);
		DMA_DeInit(DMA1_Stream7);

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	}
}

void signal_start_sampling(int adc_buffer_num) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// De-init everything just in case
	DMA_DeInit(DMA2_Stream0);
	TIM_DeInit(TIM8);
	ADC_DeInit();

	// Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// DMA
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_buffer[adc_buffer_num];
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream0, ENABLE);

	// Clear the pending interrupt just in case
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// ADC Common Init
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC1 regular channels 0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);

	// Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SystemCoreClock / SAMP_SPEED;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Disable);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	TIM_Cmd(TIM8, ENABLE);
}

void DMA2_Stream0_IRQHandler(void) {
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		DMA_DeInit(DMA2_Stream0);
		TIM_DeInit(TIM8);
		ADC_DeInit();

		process_poll(&corr_process);
	}
}
