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

#include "navigation.h"
#include "ch.h"
#include "hal.h"
#include "pos.h"
#include "utils.h"
#include "packet_handler_int.h"
#include "safety.h"

#include <math.h>
#include <string.h>

// Private variables
static float set_pos_x = 0.0;
static float set_pos_y = 0.0;
static float set_pos_z = 0.0;

static float set_pos_x_demo = 0.0;
static float set_pos_y_demo = 0.0;
static float set_pos_z_demo = 0.0;

static int override_xy = 0;
static int override_z = 0;
static int demo_started = 0;

static THD_WORKING_AREA(navigation_thread_wa, 2048);

static STATUS_MSG_t ldm[NAVIGATION_LDM_SIZE];

// Private functions
static THD_FUNCTION(navigation_thread, arg);

void navigation_init(void) {
	chThdCreateStatic(navigation_thread_wa, sizeof(navigation_thread_wa), NORMALPRIO, navigation_thread, NULL);
	for (int i = 0;i < NAVIGATION_LDM_SIZE;i++) {
		STATUS_MSG_t *msg = &ldm[i];
		memset(msg, 0, sizeof(STATUS_MSG_t));
		msg->id = -1;
	}
}

void navigation_override_xy(float x_pos, float y_pos) {
	utils_truncate_number(&x_pos, quad_config.map_lim.min_x + POS_REG_MARGIN,
			quad_config.map_lim.max_x - POS_REG_MARGIN);
	utils_truncate_number(&y_pos, quad_config.map_lim.min_y + POS_REG_MARGIN,
			quad_config.map_lim.max_y - POS_REG_MARGIN);

	set_pos_x = x_pos;
	set_pos_y = y_pos;
	set_pos_x_demo = x_pos;
	set_pos_y_demo = y_pos;
	demo_started = 0;
	override_xy = 1;
}

void navigation_override_xy_stop(void) {
	override_xy = 0;
}

void navigation_override_z(float z_pos) {
	set_pos_z = z_pos;
	set_pos_z_demo = z_pos;
	demo_started = 0;
	override_z = 1;
}

void navigation_override_z_stop(void) {
	override_z = 0;
}

void navigation_run_xy(float *roll_out, float *pitch_out, float dt) {
	static int was_override = 0;
	static float pos_i_x = 0.0;
	static float pos_i_y = 0.0;
	static float pos_prev_error_x = 0.0;
	static float pos_prev_error_y = 0.0;

	if (override_xy) {
		POS_STATE_t *pos_state = pos_get_state();

		if (was_override) {
			// Position control
			const float pos_p_gain = 0.8;
			const float pos_i_gain = 0.09;
			const float pos_d_gain = 0.6;

			float pos_error_x = set_pos_x - pos_state->px;
			float pos_error_y = set_pos_y - pos_state->py;

			float pos_p_x = pos_p_gain * pos_error_x;
			float pos_p_y = pos_p_gain * pos_error_y;

			pos_i_x += pos_error_x * pos_i_gain * dt;
			pos_i_y += pos_error_y * pos_i_gain * dt;

			float pos_d_x = ((pos_error_x - pos_prev_error_x) / dt) * pos_d_gain;
			float pos_d_y = ((pos_error_y - pos_prev_error_y) / dt) * pos_d_gain;

			pos_prev_error_x = pos_error_x;
			pos_prev_error_y = pos_error_y;

			float pos_out_x = pos_p_x + pos_i_x + pos_d_x;
			float pos_out_y = pos_p_y + pos_i_y + pos_d_y;

			const float cosy = cosf(-pos_state->yaw * M_PI / 180.0);
			const float siny = sinf(-pos_state->yaw * M_PI / 180.0);

			*roll_out = pos_out_x * cosy - pos_out_y * siny;
			*pitch_out = -pos_out_y * cosy - pos_out_x * siny;

			utils_truncate_number(roll_out, -0.6, 0.6);
			utils_truncate_number(pitch_out, -0.6, 0.6);
		} else {
			was_override = 1;
			pos_prev_error_x = 0.0;
			pos_prev_error_y = 0.0;
			pos_i_x = 0.0;
			pos_i_y = 0.0;
		}
	} else {
		was_override = 0;
	}
}

void navigation_run_z(float *throttle_out) {
	static int was_override = 0;
	static float height_i = 0.0;
	static float height_prev_error = 0.0;
	static float avg_throttle = 0.0;
	static float throttle_val = 0.0;
	static systime_t prev_last_alt_timestamp = 0;

#define THR_HEIGHT_MIN		0.3
#define THR_HEIGHT_MAX		0.8

	/*
	 * This assumes that we attempt to auto-start. Give the integrator a good
	 * initial value.
	 */
	if (*throttle_out < 0.001) {
		avg_throttle = 0.6;
	}

	avg_throttle += *throttle_out > avg_throttle ? 0.001 : -0.001;
	utils_truncate_number(&avg_throttle, THR_HEIGHT_MIN, THR_HEIGHT_MAX);

	if (override_z) {
		POS_STATE_t *pos_state = pos_get_state();

		if (was_override) {
			// Altitude control
			float height_p_gain = 0.1;
			float height_i_gain = 0.1;
			float height_d_gain = 0.14;

			if (prev_last_alt_timestamp != pos_state->pz_update_time) {
				float tim_diff = (float)(pos_state->pz_update_time - prev_last_alt_timestamp) / (float)CH_CFG_ST_FREQUENCY;
				float error = (set_pos_z - pos_state->pz);

				float height_p = error * height_p_gain;
				height_i += error * tim_diff * height_i_gain;
				utils_truncate_number(&height_i, THR_HEIGHT_MIN, THR_HEIGHT_MAX);
				float height_d = ((error - height_prev_error) / tim_diff) * height_d_gain;

				prev_last_alt_timestamp = pos_state->pz_update_time;

				throttle_val = height_p + height_i + height_d;
				utils_truncate_number(&throttle_val, THR_HEIGHT_MIN, THR_HEIGHT_MAX);

				height_prev_error = error;
			}

			*throttle_out = throttle_val;
		} else {
			was_override = 1;
			height_i = avg_throttle;

			throttle_val = avg_throttle;
			utils_truncate_number(&throttle_val, THR_HEIGHT_MIN, THR_HEIGHT_MAX);
			*throttle_out = throttle_val;

			height_prev_error = 0.0;
			prev_last_alt_timestamp = pos_state->pz_update_time;
		}
	} else {
		was_override = 0;
	}
}

void navigation_send_set_pos_to_map(void) {
	POS_STATE_t pos = *pos_get_state();
	pos.px = set_pos_x;
	pos.py = set_pos_y;
	packet_handler_send_pos(100 + quad_config.quad_id, &pos);
}

void navigation_status_received(STATUS_MSG_t *msg) {
	int found = 0;

	for (int i = 0;i < NAVIGATION_LDM_SIZE;i++) {
		STATUS_MSG_t *lmsg = &ldm[i];

		if (!found && (lmsg->id == msg->id || lmsg->id <= 0)) {
			*lmsg = *msg;
			found = 1;
		}

	}
}

STATUS_MSG_t *navigation_get_ldm_msg(int id) {
	if (id >= 0 && id < NAVIGATION_LDM_SIZE) {
		return &ldm[id];
	} else {
		return 0;
	}
}

static THD_FUNCTION(navigation_thread, arg) {
	(void) arg;
	chRegSetThreadName("Navigation");

	int was_override = 0;
	float diff_x = 0.0;
	float diff_y = 0.0;

	for(;;) {
		if (quad_config.quad_id != 0) {
			if (override_xy) {
				STATUS_MSG_t quad0;
				int found = 0;
				for (int i = 0;i < NAVIGATION_LDM_SIZE;i++) {
					if (ldm[i].id == 0) {
						quad0 = ldm[i];
						found = 1;
						break;
					}
				}

				if (found) {
					float t_diff = (float)chVTTimeElapsedSinceX(quad0.timestamp) / (float)CH_CFG_ST_FREQUENCY;
					if (!was_override) {
						if (t_diff < 1.0) {
							diff_x = set_pos_x_demo - quad0.px;
							diff_y = set_pos_y_demo - quad0.py;
							was_override = 1;
						}
					} else {
						float los_scale = 1.0;
						if (safety_get_level_of_service() != LEVEL_OF_SERVICE_HIGH) {
							los_scale = 1.3;
						}
						set_pos_x = quad0.px + diff_x * los_scale;
						set_pos_y = quad0.py + diff_y * los_scale;

						utils_truncate_number(&set_pos_x, quad_config.map_lim.min_x + POS_REG_MARGIN,
								quad_config.map_lim.max_x - POS_REG_MARGIN);
						utils_truncate_number(&set_pos_y, quad_config.map_lim.min_y + POS_REG_MARGIN,
								quad_config.map_lim.max_y - POS_REG_MARGIN);
					}
				}
			} else {
				was_override = 0;
			}
		}

		chThdSleepMilliseconds(20);
	}
}
