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
#include "stm32f4xx_conf.h"

#include "transmitter_receiver.h"

// Global variables
receiver_channel_t transmitter_receiver_channel[RECEIVER_CHANNELS] = {
		{0.0, 0, GPIOA,  0},
		{0.0, 0, GPIOA,  1},
		{0.0, 0, GPIOB,  8},
		{0.0, 0, GPIOB,  9},
		{0.0, 0, GPIOC, 10},
		{0.0, 0, GPIOC, 11}
};

// Private variables
static volatile systime_t last_update_time;

void transmitter_receiver_init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	uint16_t prescaler = 0;

	last_update_time = 0;

	for (int i = 0; i < RECEIVER_CHANNELS; i++) {
		palSetPadMode(transmitter_receiver_channel[i].port,
				transmitter_receiver_channel[i].pin, PAL_STM32_MODE_INPUT);
		extChannelEnable(&EXTD1, transmitter_receiver_channel[i].pin);
	}

	// Timer
	RCC_GetClocksFreq(&RCC_Clocks);
	prescaler = (uint16_t) ((RCC_Clocks.SYSCLK_Frequency / 2) / RECEIVER_TIMER_FREQUENCY) - 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Time base configuration
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
uint32_t transmitter_receiver_get_time_since_update(void) {
	return (systime_t)((float)chVTTimeElapsedSinceX(last_update_time) /
			((float)CH_CFG_ST_FREQUENCY / 1000.0));
}

void transmitter_receiver_isr(uint8_t pin) {
	uint16_t cnt = TIM3->CNT;
	uint16_t diff = 0;
	float v = 0.0;

	for (int i = 0; i < RECEIVER_CHANNELS; i++) {
		if (pin == transmitter_receiver_channel[i].pin) {
			if (palReadPad(transmitter_receiver_channel[i].port, transmitter_receiver_channel[i].pin)) {
				transmitter_receiver_channel[i].t_start = cnt;
			} else {
				diff = (cnt > transmitter_receiver_channel[i].t_start) ?
						cnt - transmitter_receiver_channel[i].t_start : 0xffff - (transmitter_receiver_channel[i].t_start - cnt);
				v = (float)diff * 1000.0 / (float)RECEIVER_TIMER_FREQUENCY - PPM_LENGTH_MIN;
				if (v < 0.0) {
					v = 0.0;
				} else if (v > (PPM_LENGTH_MAX - PPM_LENGTH_MIN)) {
					v = PPM_LENGTH_MAX - PPM_LENGTH_MIN;
				}
				transmitter_receiver_channel[i].value = v;
				last_update_time = chVTGetSystemTime();
			}
		}
	}
}
