/*
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

#include "ch.h"
#include "hal.h"
#include "transmitter_ppm.h"
#include <stm32f4xx_conf.h>

// Settings
#define PORT	GPIOC
#define PIN		6

// Global variables
float transmitter_ppm_channel[PPM_CHANNELS] = {0.0};

// Private variables
static volatile systime_t last_update_time;
/*
 * Table for receiver channel conversion
 * Quad expects the following:
 * Channel 0:	Roll
 *         1:	Pitch
 *         2:	Yaw
 *         3:	Throttle
 *         4:	Pot 1
 *         5:	Pot 2
 */
uint8_t channel_transform[PPM_CHANNELS] = {0, 1, 3, 2, 4, 5};


void transmitter_ppm_init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	uint16_t prescaler = 0;

	last_update_time = 0;

	palSetPadMode(PORT, PIN, PAL_STM32_MODE_INPUT);
	extChannelEnable(&EXTD1, PIN);

	/* Timer */
	RCC_GetClocksFreq(&RCC_Clocks);
	prescaler = (uint16_t) ((RCC_Clocks.SYSCLK_Frequency / 2) / RECEIVER_TIMER_FREQUENCY) - 1;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, ENABLE);
}

/**
 * Get the amount of milliseconds that have passed since
 * servo values were received the last time.
 */
uint32_t transmitter_ppm_get_time_since_update(void) {
	return (systime_t)((float)chVTTimeElapsedSinceX(last_update_time) /
			((float)CH_CFG_ST_FREQUENCY / 1000.0));
}

void transmitter_ppm_isr(uint8_t pin) {
	static uint8_t ppm_index = 0;
	static uint16_t cnt_start = 0;
	uint16_t cnt = TIM3->CNT;
	uint16_t diff = 0;
	float v;

	/* pin is not used as PPM only requires one pin */
	(void)pin;

	if (palReadPad(PORT, PIN)) {
		/* Save time for rising edge */
		cnt_start = cnt;
	}
	else {
		/* Calculate pulse length on falling edge */
		diff = (cnt > cnt_start) ? cnt - cnt_start : 0xffff - (cnt_start - cnt);

		/*
		 * Each channel has a pulse length between 0.7 and 1.7 ms, followed by a 0.3ms space.
		 * Total frame length should be 22.5ms
		 */
		if (diff > ((22.5-PPM_CHANNELS*2.0 - 2.0) * RECEIVER_TIMER_FREQUENCY/1000)) {
			/* Start of frame */
			ppm_index = 0;
		}
		else {
//			ppm_channel[ppm_index++] = (float)diff * 1000.0 / (float)RECEIVER_TIMER_FREQUENCY;
			v = (float)diff * 1000.0 / (float)RECEIVER_TIMER_FREQUENCY - PPM_LENGTH_MIN;
			v = v / (PPM_LENGTH_MAX - PPM_LENGTH_MIN);
			if (v < 0.0) {
				v = 0.0;
			} else if (v > (PPM_LENGTH_MAX - PPM_LENGTH_MIN)) {
				v = PPM_LENGTH_MAX - PPM_LENGTH_MIN;
			}

			transmitter_ppm_channel[channel_transform[ppm_index]] = v;
			last_update_time = chVTGetSystemTime();
			ppm_index++;

			if (ppm_index >= PPM_CHANNELS) {
				// this should not happen
				ppm_index = 0;
			}
		}
	}
}
