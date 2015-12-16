/*
	Copyright 2013-2015 Benjamin Vedder	benjamin@vedder.se

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
#include "packet_handler_int.h"
#include "buffer.h"
#include "pos.h"
#include "safety.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"

// Variables
static volatile float last_received_altutude;
static volatile systime_t last_update_time_altitude;

void packet_handler_int_init(void) {
	last_update_time_altitude = 0;

	// Start TIM5 as the global clock
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	PrescalerValue = (uint16_t) ((168000000 / 2) / 1000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// Prescaler configuration
	TIM_PrescalerConfig(TIM5, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_Cmd(TIM5, ENABLE);

	TIM5->CNT = 0;
}

void packet_handler_int_send_packet(uint8_t *data, uint8_t len) {
	(void)data;
	(void)len;
	// TODO
}

float packet_handler_int_get_last_altitude(void) {
	return last_received_altutude;
}

/**
 * Get the amount of milliseconds that have passed since
 * altitude values were received the last time.
 */
uint32_t packet_handler_int_get_time_since_update_altitude(void) {
	return (systime_t)((float)chVTTimeElapsedSinceX(last_update_time_altitude) /
			((float)CH_CFG_ST_FREQUENCY / 1000.0));
}

systime_t packet_handler_int_get_altitude_timestamp(void) {
	return last_update_time_altitude;
}

void packet_handler_int_process_packet(unsigned char *buffer, unsigned char len) {
	if (!len) {
		return;
	}

	// TODO: Currently, this function is run from an interrupt. It might be
	// better to do the work from a thread.

	PACKET_INT_CMD cmd = buffer[0];
	buffer++;
	len--;

	int32_t index = 0;
	int16_t int16;
	uint8_t anchor_index = 0;
	float alt_rec;

	switch (cmd) {
	case PACKET_INT_CMD_ALTITUDE:
		int16 = buffer_get_int16(buffer, &index);
		alt_rec = (float)int16 / 1000.0;

		if (alt_rec < MIN_ALTUTUDE) {
			alt_rec = MAX_ALTUTUDE;
		}

		last_received_altutude = alt_rec;
		last_update_time_altitude = chVTGetSystemTime();
		pos_correct_altitude(last_received_altutude);
		break;

	case PACKET_INT_CMD_RANGE:
		anchor_index = buffer[index++];
		pos_correct_anchor(anchor_index, (float)buffer_get_int32(buffer, &index) / 1000.0);
		break;

	case PACKET_INT_CMD_CLOCK:
		TIM5->CNT = buffer_get_uint32(buffer, &index);
		break;

	case PACKET_INT_CMD_LOS:
		safety_los_int_received(buffer[0]);
		break;

	default:
		break;
	}

}
