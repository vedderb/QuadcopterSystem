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
#include "pos.h"
#include "conf_general.h"
#include "utils.h"
#include "packet_handler.h"
#include "safety.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// Private variables
static POS_STATE_t state;
static ANCHOR_STATE_t anchors[MAX_ANCHORS];

void pos_init(void) {
	memset(&state, 0, sizeof(state));

	state.px = 1.0;
	state.py = -1.0;
	state.pz = 1.0;

	anchors[0].settings = quad_config.anchor_settings[0];
	anchors[1].settings = quad_config.anchor_settings[1];
	anchors[2].settings = quad_config.anchor_settings[2];
	anchors[3].settings = quad_config.anchor_settings[3];
}

void pos_zero_speed(void) {
	state.vx = 0.0;
	state.vy = 0.0;
}

POS_STATE_t *pos_get_state(void) {
	return &state;
}

void pos_update(float roll, float pitch, float yaw, float dt) {
	roll += state.acc_roll_err;
	pitch += state.acc_pitch_err;

	state.roll = roll;
	state.pitch = pitch;
	state.yaw = yaw;

	// Too much tilt means that this won't work anyway. Return in that case.
	if (fabsf(roll) > 45.0 || fabsf(pitch) > 45.0) {
		state.vx = 0;
		state.vy = 0;
		return;
	}

	roll = roll * M_PI / 180.0;
	pitch = pitch * M_PI / 180.0;
	yaw = yaw * M_PI / 180.0;

	const float acc_v = 9.82;
	const float cos_y = cosf(-yaw);
	const float sin_y = sinf(-yaw);

	const float dvx = acc_v * tanf(roll) * dt;
	const float dvy = -acc_v * tanf(pitch) * dt;

	state.vx += cos_y * dvx + sin_y * dvy;
	state.vy += -sin_y * dvx + cos_y * dvy;
	state.px += state.vx * dt;
	state.py += state.vy * dt;

	// Apply position and velocity limits
	if (utils_truncate_number(&state.px, quad_config.map_lim.min_x,
			quad_config.map_lim.max_x)) {
		state.vx = 0.0;
	} else {
		utils_truncate_number(&state.vx, -POS_MAX_V, POS_MAX_V);
	}

	if (utils_truncate_number(&state.py, quad_config.map_lim.min_y,
			quad_config.map_lim.max_y)) {
		state.vy = 0;
	} else {
		utils_truncate_number(&state.vy, -POS_MAX_V, POS_MAX_V);
	}

	// Exponential decay
	const float decay_factor = powf(POS_VEL_DECAY_E, dt);
	state.vx *= decay_factor;
	state.vy *= decay_factor;

	// Linear decay
	utils_step_towards(&state.vx, 0.0, POS_VEL_DECAY_L * dt);
	utils_step_towards(&state.vy, 0.0, POS_VEL_DECAY_L * dt);
}

void pos_correct_altitude(float alt) {
	state.pz = alt;
	state.pz_update_time = chVTGetSystemTime();
}

void pos_correct_anchor(int anchor_id, float anchor_distance) {
	uint32_t i;

	for (i = 0; i < MAX_ANCHORS; i++) {
		ANCHOR_STATE_t *anchor = &anchors[i];
		POS_STATE_t tmp_state;

		if (anchor->settings.id == anchor_id) {
			ANCHOR_SAFETY_INFO safety_info;
			safety_info.id = anchor->settings.id;

			const systime_t time_now = chVTGetSystemTime();
			float dt = (float)(time_now - anchor->update_time) / (float)CH_CFG_ST_FREQUENCY;
			anchor->update_time = time_now;
			safety_info.update_time = time_now;

			if (anchor_distance < 0.01) {
				anchor->last_dist = 0.0;
				safety_info.update_type = ANCHOR_UPDATE_ZERO;
				safety_update_anchor(&safety_info);
				return;
			}

			if (dt < 0.0001) {
				safety_info.update_type = ANCHOR_UPDATE_SMALL_DT;
				safety_update_anchor(&safety_info);
				return;
			}

			chSysLock();
			tmp_state = state;
			chSysUnlock();

			const float anchor_pos_gain_p = 0.3;
			const float anchor_pos_gain_i = 0.0;
			const float anchor_pos_gain_d = 0.02;

			const float anchor_vel_gain_p = 0.1;
			const float anchor_vel_gain_i = 0.0;
			const float anchor_vel_gain_d = 0.2;

			const float anchor_acc_gain_p = 0.2;// 0.2
			const float anchor_acc_gain_i = 0.0;
			const float anchor_acc_gain_d = 0.05;// 0.8

			const float anchor_max_corr = 0.5;
			const float anchor_max_tilt = 6.0;

			const float da_x = tmp_state.px - anchor->settings.px;
			const float da_y = tmp_state.py - anchor->settings.py;
			const float da_z = tmp_state.pz - anchor->settings.pz;
			const float da = sqrtf(da_x*da_x + da_y*da_y + da_z*da_z);

			float error = da - anchor_distance;

			if (fabsf(error) > 0.2) {
				anchor->wrong_iterations++;
				if (anchor->wrong_iterations < 3 && !anchor->out_of_pos) {
					safety_info.update_type = ANCHOR_UPDATE_BIG_CORR;
					safety_update_anchor(&safety_info);
					return;
				} else {
					anchor->out_of_pos = 1;
					safety_info.update_type = ANCHOR_UPDATE_OUT_OF_POS;
					safety_update_anchor(&safety_info);
				}
			} else {
				anchor->wrong_iterations = 0;
				anchor->out_of_pos = 0;
				safety_info.update_type = ANCHOR_UPDATE_OK;
				safety_update_anchor(&safety_info);
			}

			utils_truncate_number(&error, -anchor_max_corr, anchor_max_corr);

			anchor->update_time = time_now;
			anchor->last_dist = anchor_distance;

			const float comp_factor = error / da;

			const float pcx = da_x * comp_factor;
			const float pcy = da_y * comp_factor;

			const float pcx_p = pcx * anchor_pos_gain_p;
			const float pcy_p = pcy * anchor_pos_gain_p;

			anchor->corr_pcxI += pcx * anchor_pos_gain_i * dt;
			anchor->corr_pcyI += pcy * anchor_pos_gain_i * dt;
			utils_truncate_number(&anchor->corr_pcxI, -0.2, 0.2);
			utils_truncate_number(&anchor->corr_pcyI, -0.2, 0.2);

			const float pcx_d = (pcx - anchor->corr_pcxLast) * anchor_pos_gain_d / dt;
			const float pcy_d = (pcy - anchor->corr_pcyLast) * anchor_pos_gain_d / dt;
			anchor->corr_vcxLast = pcx;
			anchor->corr_vcyLast = pcy;

			const float pcx_out = pcx_p + anchor->corr_pcxI + pcx_d;
			const float pcy_out = pcy_p + anchor->corr_pcyI + pcy_d;

			tmp_state.px -= pcx_out;
			tmp_state.py -= pcy_out;

			const float cosy = cosf(-tmp_state.yaw * M_PI / 180.0);
			const float siny = sinf(-tmp_state.yaw * M_PI / 180.0);

			const float vcx = da_x * comp_factor;
			const float vcy = da_y * comp_factor;

			const float vcx_p = vcx * anchor_vel_gain_p;
			const float vcy_p = vcy * anchor_vel_gain_p;

			anchor->corr_vcxI += vcx * anchor_vel_gain_i * dt;
			anchor->corr_vcyI += vcy * anchor_vel_gain_i * dt;
			utils_truncate_number(&anchor->corr_vcxI, -1.0, 1.0);
			utils_truncate_number(&anchor->corr_vcyI, -1.0, 1.0);

			const float vcx_d = (vcx - anchor->corr_vcxLast) * anchor_vel_gain_d / dt;
			const float vcy_d = (vcy - anchor->corr_vcyLast) * anchor_vel_gain_d / dt;
			anchor->corr_vcxLast = vcx;
			anchor->corr_vcyLast = vcy;

			const float vcx_out = vcx_p + anchor->corr_vcxI + vcx_d;
			const float vcy_out = vcy_p + anchor->corr_vcyI + vcy_d;

			tmp_state.vx -= vcx_out;
			tmp_state.vy -= vcy_out;

			const float acx = da_x * comp_factor;
			const float acy = da_y * comp_factor;

			const float acx_p = acx * anchor_acc_gain_p;
			const float acy_p = acy * anchor_acc_gain_p;

			anchor->corr_acxI += acx * anchor_acc_gain_i * dt;
			anchor->corr_acyI += acy * anchor_acc_gain_i * dt;
			utils_truncate_number(&anchor->corr_acxI, -1.0, 1.0);
			utils_truncate_number(&anchor->corr_acyI, -1.0, 1.0);

			const float acx_d = (acx - anchor->corr_acxLast) * anchor_acc_gain_d / dt;
			const float acy_d = (acy - anchor->corr_acyLast) * anchor_acc_gain_d / dt;
			anchor->corr_acxLast = acx;
			anchor->corr_acyLast = acy;

			const float acx_out = acx_p + anchor->corr_acxI + acx_d;
			const float acy_out = acy_p + anchor->corr_acyI + acy_d;

			tmp_state.acc_roll_err -= acx_out * cosy - acy_out * siny;
			tmp_state.acc_pitch_err += acy_out * cosy + acx_out * siny;

			utils_truncate_number(&tmp_state.acc_roll_err, -anchor_max_tilt, anchor_max_tilt);
			utils_truncate_number(&tmp_state.acc_pitch_err, -anchor_max_tilt, anchor_max_tilt);

			chSysLock();
			state = tmp_state;
			chSysUnlock();
			break;
		}
	}
}

ANCHOR_STATE_t *pos_get_anchor(int id) {
	ANCHOR_STATE_t *retval = 0;
	uint32_t i;

	for (i = 0; i < MAX_ANCHORS; i++) {
		if (anchors[i].settings.id == id) {
			retval = &anchors[i];
			break;
		}
	}

	return retval;
}
