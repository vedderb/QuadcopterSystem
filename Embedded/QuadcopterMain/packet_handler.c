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

#include "packet_handler.h"
#include "pwm_esc.h"
#include "utils.h"
#include "actuator.h"
#include "led.h"
#include "rf_control.h"
#include "control.h"
#include "buffer.h"
#include "mpu9150.h"
#include "packet_handler_int.h"
#include "navigation.h"
#include "safety.h"
#include <stdlib.h>
#include <stdio.h>
#include "led_external.h"
#include "ch.h"

// Private variables
static THD_WORKING_AREA(packet_handler_thread_wa, 512);
static THD_WORKING_AREA(zero_thread_wa, 1024);
static volatile float pwm_channels[PWM_RX_CH_NUM];
static volatile systime_t last_stick_update_time;
static volatile float center_roll;
static volatile float center_pitch;
static volatile float center_yaw;
static volatile float center_roll_s;
static volatile float center_pitch_s;
static volatile float center_yaw_s;
static volatile int center_samples;
static virtual_timer_t emergency_stop_reset_timer;
static volatile bool zero_thread_running;

// Private functions
static THD_FUNCTION(packet_handler_thread, arg);
static THD_FUNCTION(zero_thread, arg);
static void emergency_stop_reset(void *p);

void packet_handler_init(void) {
	last_stick_update_time = 0;
	center_roll = CENTER_ROLL;
	center_pitch = CENTER_PITCH;
	center_yaw = CENTER_YAW;

	center_roll_s = 0.0;
	center_pitch_s = 0.0;
	center_yaw_s = 0.0;
	center_samples = 0;

	zero_thread_running = false;

	packet_handler_zero_channels();

	chThdCreateStatic(packet_handler_thread_wa, sizeof(packet_handler_thread_wa),
			NORMALPRIO, packet_handler_thread, NULL);
}

static THD_FUNCTION(packet_handler_thread, arg) {
	(void)arg;

	chRegSetThreadName("Packet Handler");

	for(;;) {
		if (packet_handler_get_time_since_stick_update() > 500) {
			led_write(LED_GREEN, 0);
		} else {
			led_write(LED_GREEN, 1);
		}

		chThdSleepMilliseconds(10);
	}
}

static THD_FUNCTION(zero_thread, arg) {
	(void)arg;

	zero_thread_running = true;
	chRegSetThreadName("Zero Command");

	led_write(LED_RED, 1);

	packet_handler_printf("Sampling the center offsets for 50 iterations...");

	int res = packet_handler_sample_centers(50, 4000);

	if (res) {
		packet_handler_printf("Offset Roll : %.3f\n"
				"Offset Pitch: %.3f\n"
				"Offset Yaw  : %.3f\n",
				(double)center_roll, (double)center_pitch, (double)center_yaw);
	} else {
		packet_handler_printf("Sampling the centers timed out\n");
	}

	packet_handler_printf("Sampling the gyro offsets for 100 iterations...");

	mpu9150_sample_gyro_offsets(100);

	packet_handler_printf("Offset X: %d\n"
			"Offset Y: %d\n"
			"Offset Z: %d\n",
			mpu9150_gyro_offsets[0],
			mpu9150_gyro_offsets[1],
			mpu9150_gyro_offsets[2]);

	control_update_initial_att();

	led_write(LED_RED, 0);
	zero_thread_running = false;
}

static void emergency_stop_reset(void *p) {
	(void)p;
	quad_config.emergency_stop = false;
}

void packet_handler_send_packet(uint8_t *data, uint8_t len) {
	rf_control_send_packet(data, len);
}

void packet_handler_process_packet(uint8_t *data, uint8_t len) {
	uint8_t pwm_channel = 0;
	uint8_t pulse_length = 0;
	static uint8_t send_buffer[200];
	int32_t send_index = 0;
	CONTROL_PARAMETERS_t *ctrl_param;
	PACKET_CMD_t cmd;
	uint8_t id;
	int32_t tmp_i32;
	int32_t data_ind;
	float roll, pitch, yaw;
	float accel[3];
	float gyro[3];
	float mag[3];
	float tmp_f;
	POS_STATE_t *pos_state;
	ANCHOR_STATE_t *anchor;
	SAFETY_MSG_t safety_msg;
	STATUS_MSG_t status_msg;

	id = data[0];
	data++;
	len--;

	cmd = data[0];
	data++;
	len--;

	if (id == quad_config.quad_id || id == QUAD_ID_ALL) {
		switch (cmd) {
		// Set stick input
		case PACKET_CMD_SET_STICK_INPUT:
			if (!quad_config.emergency_stop) {
				for (int i = 0;i < len / 2;i++) {
					pwm_channel = data[2 * i];
					pulse_length = data[2 * i + 1];

					if (pwm_channel <= PWM_RX_CH_NUM) {
						pwm_channels[pwm_channel] = (float)pulse_length / 255.0;
						last_stick_update_time = chVTGetSystemTime();
					}
				}

				if (center_samples > 0) {
					center_roll_s += pwm_channels[CH_ROLL];
					center_pitch_s += pwm_channels[CH_PITCH];
					center_yaw_s += pwm_channels[CH_YAW];
					center_samples--;
				}
			}
			break;

		case PACKET_CMD_GET_CONTROL_PARAMETERS:
			if (len != 0) {
				break;
			}

			ctrl_param = control_get_pid_parameters();
			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_CONTROL_PARAMETERS;

			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->roll_p_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->roll_i_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->roll_d_gain_process * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->roll_d_gain_error * 100000.0), &send_index);

			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->pitch_p_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->pitch_i_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->pitch_d_gain_process * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->pitch_d_gain_error * 100000.0), &send_index);

			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->yaw_p_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->yaw_i_gain * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->yaw_d_gain_process * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(ctrl_param->yaw_d_gain_error * 100000.0), &send_index);

			packet_handler_send_packet(send_buffer, send_index);

			tmp_f = control_get_last_iteration_delay_ms();
			packet_handler_printf("Sample duration: %.2f ms", (double)tmp_f);
			packet_handler_printf("Rate: %.2f Hz", (double)(1000.0 / tmp_f));
			packet_handler_printf("Failed reads: %d", mpu9150_get_failed_reads());
			packet_handler_printf("Failed mag reads: %d\n", mpu9150_get_failed_mag_reads());
			break;

		case PACKET_CMD_SET_CONTROL_PARAMETERS:
			ctrl_param = control_get_pid_parameters();

			data_ind = 0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->roll_p_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->roll_i_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->roll_d_gain_process = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->roll_d_gain_error = (float)tmp_i32 / 100000.0;

			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->pitch_p_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->pitch_i_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->pitch_d_gain_process = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->pitch_d_gain_error = (float)tmp_i32 / 100000.0;

			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->yaw_p_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->yaw_i_gain = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->yaw_d_gain_process = (float)tmp_i32 / 100000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			ctrl_param->yaw_d_gain_error = (float)tmp_i32 / 100000.0;
			break;

		case PACKET_CMD_GET_ORIENTATION:
			if (len != 0) {
				break;
			}
			control_get_orientation_now(&roll, &pitch, &yaw);

			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_ORIENTATION;

			buffer_append_int32(send_buffer, (int32_t)(roll * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(pitch * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(yaw * 100000.0), &send_index);

			packet_handler_send_packet(send_buffer, send_index);
			break;

		case PACKET_CMD_RESET_GYRO_STICKS:
			if (!zero_thread_running) {
				chThdCreateStatic(zero_thread_wa, sizeof(zero_thread_wa),
						NORMALPRIO, zero_thread, NULL);
			}
			break;

		case PACKET_CMD_RESET_ORIENTATION:
			control_reset_orientation_offsets();
			break;

		case PACKET_CMD_GET_RAW_IMU:
			if (len != 0) {
				break;
			}

			mpu9150_get_accel_gyro_mag(accel, gyro, mag);
			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_RAW_IMU;

			buffer_append_int32(send_buffer, (int32_t)(accel[0] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(accel[1] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(accel[2] * 100000.0), &send_index);

			buffer_append_int32(send_buffer, (int32_t)(gyro[0] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(gyro[1] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(gyro[2] * 100000.0), &send_index);

			buffer_append_int32(send_buffer, (int32_t)(mag[0] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(mag[1] * 100000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(mag[2] * 100000.0), &send_index);

			packet_handler_send_packet(send_buffer, send_index);
			break;

		case PACKET_CMD_OVERRIDE_POWER:
			if (!quad_config.emergency_stop) {
				data_ind = 0;
				while (data_ind < len) {
					tmp_i32 = buffer_get_int32(data, &data_ind);
					control_override_power((data_ind / 4) - 1, (float)tmp_i32 / 100000.0);
				}
			}
			break;

		case PACKET_CMD_GET_POS:
			if (len != 0) {
				break;
			}

			packet_handler_send_pos(quad_config.quad_id, pos_get_state());
//			navigation_send_set_pos_to_map();
			break;

		case PACKET_CMD_SET_POS:
			pos_state = pos_get_state();

			data_ind = 0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			pos_state->px = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			pos_state->py = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			pos_state->vx = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			pos_state->vy = (float)tmp_i32 / 10000.0;

			tmp_i32 = buffer_get_int32(data, &data_ind);
			tmp_f = (float)tmp_i32 / 100000.0;
			control_set_yaw(tmp_f);
			pos_state->yaw = tmp_f;
			break;

		case PACKET_CMD_GET_ALTITUDE:
			if (len != 0) {
				break;
			}

			tmp_f = packet_handler_int_get_last_altitude();

			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_ALTITUDE;

			buffer_append_int32(send_buffer, (int32_t)(tmp_f * 100000.0), &send_index);

			packet_handler_send_packet(send_buffer, send_index);
			break;

		case PACKET_CMD_GET_ULTRA_DIST:
			if (len != 1) {
				break;
			}

			anchor = pos_get_anchor(data[0]);
			if (anchor == 0) {
				break;
			}

			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_ULTRA_DIST;
			send_buffer[send_index++] = anchor->settings.id;
			buffer_append_int32(send_buffer, (int32_t)(anchor->last_dist * 100000.0), &send_index);

			packet_handler_send_packet(send_buffer, send_index);
			break;

		case PACKET_CMD_RADIO_ALIVE:
			last_stick_update_time = chVTGetSystemTime();
			break;

		case PACKET_CMD_SAFETY_MSG:
			data_ind = 0;

			safety_msg.id = data[data_ind++];
			safety_msg.timestamp = chVTGetSystemTime();
			safety_msg.los_local = data[data_ind++];
			safety_msg.los_global = data[data_ind++];

			safety_msg_received(&safety_msg);
			break;

		case PACKET_CMD_STATUS_MSG:
			data_ind = 0;

			status_msg.id = data[data_ind++];
			status_msg.timestamp = chVTGetSystemTime();
			status_msg.fw_version_major = data[data_ind++];
			status_msg.fw_version_minor = data[data_ind++];
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.px = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.py = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.pz = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.roll = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.pitch = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.yaw = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.vx = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			status_msg.vy = (float)tmp_i32 / 10000.0;

			navigation_status_received(&status_msg);
			break;

		case PACKET_CMD_GET_ANCHORS:
			if (len != 0) {
				break;
			}

			send_index = 0;
			send_buffer[send_index++] = quad_config.quad_id;
			send_buffer[send_index++] = PACKET_CMD_GET_ANCHORS;

			buffer_append_int32(send_buffer, (int32_t)(quad_config.map_lim.min_x * 10000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(quad_config.map_lim.max_x * 10000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(quad_config.map_lim.min_y * 10000.0), &send_index);
			buffer_append_int32(send_buffer, (int32_t)(quad_config.map_lim.max_y * 10000.0), &send_index);

			for (int i = 0;i < MAX_ANCHORS;i++) {
				send_buffer[send_index++] = quad_config.anchor_settings[i].id;
				buffer_append_int32(send_buffer, (int32_t)(quad_config.anchor_settings[i].px * 10000.0), &send_index);
				buffer_append_int32(send_buffer, (int32_t)(quad_config.anchor_settings[i].py * 10000.0), &send_index);
				buffer_append_int32(send_buffer, (int32_t)(quad_config.anchor_settings[i].pz * 10000.0), &send_index);
			}

			packet_handler_send_packet(send_buffer, send_index);
			break;

		case PACKET_CMD_SET_ANCHORS:
			data_ind = 0;

			tmp_i32 = buffer_get_int32(data, &data_ind);
			quad_config.map_lim.min_x = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			quad_config.map_lim.max_x = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			quad_config.map_lim.min_y = (float)tmp_i32 / 10000.0;
			tmp_i32 = buffer_get_int32(data, &data_ind);
			quad_config.map_lim.max_y = (float)tmp_i32 / 10000.0;

			for (int i = 0;(i < MAX_ANCHORS) && (data_ind < len);i++) {
				quad_config.anchor_settings[i].id = data[data_ind++];
				tmp_i32 = buffer_get_int32(data, &data_ind);
				quad_config.anchor_settings[i].px = (float)tmp_i32 / 10000.0;
				tmp_i32 = buffer_get_int32(data, &data_ind);
				quad_config.anchor_settings[i].py = (float)tmp_i32 / 10000.0;
				tmp_i32 = buffer_get_int32(data, &data_ind);
				quad_config.anchor_settings[i].pz = (float)tmp_i32 / 10000.0;
			}

			conf_general_store_quad_configuration(&quad_config);
			pos_init();
			break;

		case PACKET_CMD_EMERGENCY_STOP:
			packet_handler_zero_channels();
			quad_config.emergency_stop = true;

			chSysLock();
			if (chVTIsArmedI(&emergency_stop_reset_timer)) {
				chVTResetI(&emergency_stop_reset_timer);
			}
			chVTSetI(&emergency_stop_reset_timer, S2ST(EMERGENCY_STOP_TIME), emergency_stop_reset, 0);
			chSysUnlock();
			break;

		case PACKET_CMD_SET_YAW:
			pos_state = pos_get_state();
			data_ind = 0;

			tmp_i32 = buffer_get_int32(data, &data_ind);
			tmp_f = (float)tmp_i32 / 100000.0;
			control_set_yaw(tmp_f);
			pos_state->yaw = tmp_f;
			break;

		case PACKET_CMD_SET_LED_EXT_IND_LANDED:
			led_external_set_landed_indication(data[0]);
			break;

		default:
			break;
		}
	}
}

void packet_handler_send_pos(int id, POS_STATE_t *pos) {
	uint8_t send_buffer[50];
	int32_t send_index = 0;
	int16_t pos_type;

	send_index = 0;
	send_buffer[send_index++] = id;
	send_buffer[send_index++] = PACKET_CMD_GET_POS;

	// Position type
	pos_type = POS_TYPE_NORMAL;
	pos_type |= POS_MASK_FOLLOWING;
	buffer_append_int16(send_buffer, pos_type, &send_index);

	buffer_append_int32(send_buffer, (int32_t)(pos->px * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(pos->py * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(pos->vx * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(pos->vy * 10000.0), &send_index);

	buffer_append_int32(send_buffer, (int32_t)(pos->roll * 100000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(pos->pitch * 100000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(pos->yaw * 100000.0), &send_index);

	packet_handler_send_packet(send_buffer, send_index);
}

void packet_handler_send_safety_msg(int id, SAFETY_MSG_t *msg) {
	uint8_t send_buffer[25];
	int32_t send_index = 0;

	send_index = 0;
	send_buffer[send_index++] = id;
	send_buffer[send_index++] = PACKET_CMD_SAFETY_MSG;
	send_buffer[send_index++] = msg->id;
	send_buffer[send_index++] = msg->los_local;
	send_buffer[send_index++] = msg->los_global;

	packet_handler_send_packet(send_buffer, send_index);
}

void packet_handler_send_status_msg(int id, STATUS_MSG_t *msg) {
	uint8_t send_buffer[50];
	int32_t send_index = 0;

	send_index = 0;
	send_buffer[send_index++] = id;
	send_buffer[send_index++] = PACKET_CMD_STATUS_MSG;
	send_buffer[send_index++] = msg->id;
	send_buffer[send_index++] = msg->fw_version_major;
	send_buffer[send_index++] = msg->fw_version_minor;
	buffer_append_int32(send_buffer, (int32_t)(msg->px * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->py * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->pz * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->roll * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->pitch * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->yaw * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->vx * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->vy * 10000.0), &send_index);
	buffer_append_int32(send_buffer, (int32_t)(msg->vbat * 10000.0), &send_index);
	for (int i = 0;i < 4;i++) {
		buffer_append_uint16(send_buffer, msg->adc_in[i], &send_index);
	};

	packet_handler_send_packet(send_buffer, send_index);
}

void packet_handler_cmd_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;

	for(int i = 0;i < 500; i++) {
		for(int j = 0;j < PWM_RX_CH_NUM;j++) {
			chprintf(chp, "Channel %d: %.2f\r\n", j, (double)packet_handler_get_channel(j));
		}
		chprintf(chp, "\r\n");

		for(int j = 0;j < PWM_RX_CH_NUM;j++) {
			chprintf(chp, "Mapped %d: %.2f\r\n", j, (double)packet_handler_get_channel_mapped(j));
		}
		chprintf(chp, "\r\n");

		chThdSleepMilliseconds(20);
	}
}

void packet_handler_cmd_sample_centers(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;

	chprintf(chp, "Sampling the center offsets for 50 iterations...\r\n");

	int res = packet_handler_sample_centers(50, 4000);

	if (res) {
		chprintf(chp,
				"Offset Roll : %.2f\r\n"
				"Offset Pitch: %.2f\r\n"
				"Offset Yaw  : %.2f\r\n\r\n",
				(double)center_roll, (double)center_pitch, (double)center_yaw);
	} else {
		chprintf(chp, "Sampling the centers timed out\r\n");
	}
}

/**
 * Get the amount of milliseconds that have passed since
 * servo values were received the last time.
 */
uint32_t packet_handler_get_time_since_stick_update(void) {
	return (systime_t)((float)chVTTimeElapsedSinceX(last_stick_update_time) /
			((float)CH_CFG_ST_FREQUENCY / 1000.0));
}

int packet_handler_sample_centers(int samples, uint32_t timeout_ms) {
	center_roll_s = 0.0;
	center_pitch_s = 0.0;
	center_yaw_s = 0.0;
	center_samples = samples;

	for(;;) {
		if (center_samples > 0) {
			chThdSleepMilliseconds(1);
			timeout_ms--;
		} else {
			break;
		}

		if (timeout_ms == 0) {
			return 0;
		}
	}

	center_roll = center_roll_s / samples;
	center_pitch = center_pitch_s / samples;
	center_yaw = center_yaw_s / samples;

	return 1;
}

void packet_handler_printf(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	static char print_buffer[255];
	int len;
	print_buffer[0] = quad_config.quad_id;
	print_buffer[1] = PACKET_CMD_PRINT;
	len = vsnprintf(print_buffer+2, 253, format, arg);
	va_end (arg);
	if (len > 0) {
		packet_handler_send_packet((unsigned char*)print_buffer, (len<253)? len+2: 255);
	}
	return;
}

float packet_handler_get_channel(int channel) {
	if (channel < 0 || channel >= PWM_RX_CH_NUM) {
		return 0.0;
	}

	return pwm_channels[channel];
}

void packet_handler_zero_channels(void) {
	pwm_channels[CH_THROTTLE] = 0.0;
	pwm_channels[CH_ROLL] = CENTER_ROLL;
	pwm_channels[CH_PITCH] = CENTER_PITCH;
	pwm_channels[CH_YAW] = CENTER_YAW;
	pwm_channels[CH_POT_L] = 0.0;
	pwm_channels[CH_POT_R] = 0.0;
}

/*
 * Get mapped, centered, scaled and inversion-corrected values.
 *
 * Throttle: [0.0, 1.0]
 * Roll:     [-1.0, 1.0] Center 0.0
 * Pitch:    [-1.0, 1.0] Center 0.0
 * Yaw:      [-1.0, 1.0] Center 0.0
 * Pot L:    [0.0, 1.0]
 * Pot R:    [0.0, 1.0]
 */
float packet_handler_get_channel_mapped(int channel) {
	float retval = 0.0;
	float ch = 0.0;

	switch (channel) {
	case CH_THROTTLE:
		retval = utils_map(pwm_channels[CH_THROTTLE],
					LIM_THROTTLE_MIN, LIM_THROTTLE_MAX, 0.0, 1.0);
		utils_truncate_number(&retval, 0.0, 1.0);
		if (INV_THROTTLE) {
			retval = (1.0 - retval);
		}
		break;

	case CH_ROLL:
		ch = pwm_channels[CH_ROLL];

		if (ch < center_roll) {
			retval = utils_map(ch,
					LIM_ROLL_MIN, center_roll, -1.0, 0.0);
		} else {
			retval = utils_map(ch,
					center_roll, LIM_ROLL_MAX, 0.0, 1.0);
		}

		utils_truncate_number(&retval, -1.0, 1.0);

		if (INV_ROLL) {
			retval = -retval;
		}
		break;

	case CH_PITCH:
		ch = pwm_channels[CH_PITCH];

		if (ch < center_pitch) {
			retval = utils_map(ch,
					LIM_PITCH_MIN, center_pitch, -1.0, 0.0);
		} else {
			retval = utils_map(ch,
					center_pitch, LIM_PITCH_MAX, 0.0, 1.0);
		}

		utils_truncate_number(&retval, -1.0, 1.0);

		if (INV_PITCH) {
			retval = -retval;
		}
		break;

	case CH_YAW:
		ch = pwm_channels[CH_YAW];

		if (ch < center_yaw) {
			retval = utils_map(ch,
					LIM_YAW_MIN, center_yaw, -1.0, 0.0);
		} else {
			retval = utils_map(ch,
					center_yaw, LIM_YAW_MAX, 0.0, 1.0);
		}

		utils_truncate_number(&retval, -1.0, 1.0);

		if (INV_YAW) {
			retval = -retval;
		}
		break;

	case CH_POT_L:
		retval = utils_map(pwm_channels[CH_POT_L],
				LIM_POT_L_MIN, LIM_POT_L_MAX, 0.0, 1.0);
		utils_truncate_number(&retval, 0.0, 1.0);
		if (INV_POT_L) {
			retval = (1.0 - retval);
		}
		break;

	case CH_POT_R:
		retval = utils_map(pwm_channels[CH_POT_R],
				LIM_POT_R_MIN, LIM_POT_R_MAX, 0.0, 1.0);
		utils_truncate_number(&retval, 0.0, 1.0);
		if (INV_POT_R) {
			retval = (1.0 - retval);
		}
		break;

	default:
		break;
	}

	return retval;
}
