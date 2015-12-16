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

#include "pwm_esc.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"

/* GPIOC configuration for hardware-based PWM:
 * 		TIM3 CH1 (PC6) (P18)
 * 		TIM3 CH2 (PC7) (P17)
 * 		TIM3 CH3 (PC8) (P16)
 * 		TIM3 CH4 (PC9) (P15)
 */

#include "pwm_esc.h"

#define ESC_UPDATE_RATE		400	// Hz
#define TIM_CLOCK			10e6 // Hz

static PWMConfig pwmcfg3 = {
		TIM_CLOCK, // Clock frequency.
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE), // TOP cnt
		NULL, // Callback
		{
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

#define ALL_CHANNELS		0xFF

void pwm_esc_init(void) {
	pwmStart(&PWMD3, &pwmcfg3);

	palSetPadMode(GPIOC, 6,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOC, 7,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOC, 8,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOC, 9,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);

	pwm_esc_set_all(0);
}

void pwm_esc_set_all(uint8_t pulse_width) {
	pwm_esc_set(ALL_CHANNELS, pulse_width);
}

void pwm_esc_set(uint8_t channel, uint8_t pulse_width) {
	uint32_t cnt_val;

	// Always set zero if emergency stop is set.
	if (quad_config.emergency_stop) {
		cnt_val = (uint32_t)TIM_CLOCK/1000L + (uint32_t)TIM_CLOCK*0/(1000*255);
	} else {
		cnt_val = (uint32_t)TIM_CLOCK/1000L + (uint32_t)TIM_CLOCK*pulse_width/(1000*255);
	}

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD3, 0, cnt_val);
		break;

	case 1:
		pwmEnableChannel(&PWMD3, 1, cnt_val);
		break;

	case 2:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		break;

	case 3:
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD3, 0, cnt_val);
		pwmEnableChannel(&PWMD3, 1, cnt_val);
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		break;

	default:
		break;
	}
}
