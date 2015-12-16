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

#ifndef PACKET_HANDLER_H_
#define PACKET_HANDLER_H_

#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "chprintf.h"
#include "pos.h"
#include "safety.h"

// Settings
#define PWM_RX_CH_NUM			6

// Channel mapping
#define CH_THROTTLE				3
#define CH_ROLL					0
#define CH_PITCH				1
#define CH_YAW					2
#define CH_POT_L				4
#define CH_POT_R				5

// Limits
#define LIM_THROTTLE_MIN		0.13
#define LIM_THROTTLE_MAX		0.88
#define LIM_ROLL_MIN			0.11
#define LIM_ROLL_MAX			0.90
#define LIM_PITCH_MIN			0.11
#define LIM_PITCH_MAX			0.88
#define LIM_YAW_MIN				0.13
#define LIM_YAW_MAX				0.93
#define LIM_POT_L_MIN			0.0
#define LIM_POT_L_MAX			1.0
#define LIM_POT_R_MIN			0.0
#define LIM_POT_R_MAX			1.0

// Inversion
#define INV_THROTTLE			0
#define INV_ROLL				0
#define INV_PITCH				0
#define INV_YAW					0
#define INV_POT_L				0
#define INV_POT_R				0

// Default center values (may be overridden after calibration)
#define CENTER_ROLL				0.5
#define CENTER_PITCH			0.5
#define CENTER_YAW				0.5

// Functions
void packet_handler_init(void);
void packet_handler_send_packet(uint8_t *data, uint8_t len);
void packet_handler_process_packet(uint8_t *data, uint8_t len);
void packet_handler_send_pos(int id, POS_STATE_t *pos);
void packet_handler_send_safety_msg(int id, SAFETY_MSG_t *msg);
void packet_handler_send_status_msg(int id, STATUS_MSG_t *msg);
void packet_handler_cmd_print(BaseSequentialStream *chp, int argc, char *argv[]);
void packet_handler_cmd_sample_centers(BaseSequentialStream *chp, int argc, char *argv[]);
uint32_t packet_handler_get_time_since_stick_update(void);
int packet_handler_sample_centers(int samples, uint32_t timeout_ms);
void packet_handler_printf(char* format, ...);
void packet_handler_zero_channels(void);
float packet_handler_get_channel(int channel);
float packet_handler_get_channel_mapped(int channel);

#endif /* PACKET_HANDLER_H_ */
