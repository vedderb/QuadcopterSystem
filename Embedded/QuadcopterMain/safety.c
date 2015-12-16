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

#include "safety.h"
#include "ch.h"
#include "pwm_esc.h"
#include "navigation.h"
#include "packet_handler_int.h"
#include <string.h>
#include <math.h>

// Settings
#define ANCH_MAX_IDS					5
#define ANCH_UPDATE_LEN 				30
#define ANCH_MAX_UPDATE_DELAY			1.5
#define SDM_SIZE						10
#define LOS_PERIOD						3.0
#define LOS_BROADCAST_TIME				1.0
#define LOS_BROADCAST_DECISION_TIME		0.5

// Private variables
static ANCHOR_SAFETY_INFO anchor_updates[ANCH_UPDATE_LEN];
static int anchor_update_write;
static THD_WORKING_AREA(safety_thread_wa, 1024);
static LEVEL_OF_SERVICE_t level_of_service;
static LEVEL_OF_SERVICE_t level_of_service_rec;
static LEVEL_OF_SERVICE_t level_of_service_int;
static LEVEL_OF_SERVICE_t los_local;
static LEVEL_OF_SERVICE_t los_global;
static SAFETY_MSG_t sdm[SDM_SIZE];

// Private functions
static THD_FUNCTION(safety_thread, arg);

void safety_init(void) {
	anchor_update_write = 0;
	level_of_service = LEVEL_OF_SERVICE_HIGH;
	level_of_service_rec = LEVEL_OF_SERVICE_HIGH;
	level_of_service_int = LEVEL_OF_SERVICE_HIGH;
	los_local = LEVEL_OF_SERVICE_HIGH;
	los_global = LEVEL_OF_SERVICE_HIGH;
	memset(anchor_updates, 0, sizeof(anchor_updates));

	for (int i = 0;i < SDM_SIZE;i++) {
		SAFETY_MSG_t *msg = &sdm[i];
		memset(msg, 0, sizeof(SAFETY_MSG_t));
		msg->id = -1;
	}

	chThdCreateStatic(safety_thread_wa, sizeof(safety_thread_wa), NORMALPRIO, safety_thread, NULL);
}

static THD_FUNCTION(safety_thread, arg) {
	(void) arg;
	chRegSetThreadName("Safety");

	for(;;) {
		// If emergency stop is used
		if (quad_config.emergency_stop) {
			pwm_esc_set_all(0);
		}

		// Check anchors
		int anchor_ok_updates = 0;
		int anchor_not_ok_updates = 0;
		int anch_time_id[ANCH_MAX_IDS];
		int anch_time_id_ind = 0;
		const systime_t time_now = chVTGetSystemTime();

		for (int i = 0;i < ANCH_UPDATE_LEN;i++) {
			ANCHOR_SAFETY_INFO *anch = &anchor_updates[i];
			float dt = (float)(time_now - anch->update_time) / (float)CH_CFG_ST_FREQUENCY;

			if (anch->updated) {
				// Check how many anchors had at least one good update within
				// the latest MAX_UPDATE_DELAY seconds.
				if (dt < ANCH_MAX_UPDATE_DELAY && (anch->update_type == ANCHOR_UPDATE_OK)) {

					int present = 0;
					for (int j = 0;j < anch_time_id_ind;j++) {
						if (anch_time_id[j] == anch->id) {
							present = 1;
						}
					}
					if (!present) {
						anch_time_id[anch_time_id_ind] = anch->id;
						if (anch_time_id_ind < ANCH_MAX_IDS) {
							anch_time_id_ind++;
						}
					}
				}

				// Count how many anchors are OK and how many not
				if (anch->update_type == ANCHOR_UPDATE_OK ||
						anch->update_type == ANCHOR_UPDATE_ZERO) {
					anchor_ok_updates++;
				} else {
					anchor_not_ok_updates++;
				}
			}
		}

		// Compute level of service
		if (anch_time_id_ind > 3) {
			if (anchor_not_ok_updates < 15) {
				level_of_service = LEVEL_OF_SERVICE_HIGH;
			} else {
				level_of_service = LEVEL_OF_SERVICE_MEDIUM;
			}
		} else if (anch_time_id_ind == 3) {
			if (anchor_not_ok_updates < 15) {
				level_of_service = LEVEL_OF_SERVICE_MEDIUM;
			} else {
				level_of_service = LEVEL_OF_SERVICE_LOW;
			}
		} else {
			level_of_service = LEVEL_OF_SERVICE_LOW;
		}

		// Internal communication
		if ((int)level_of_service > (int)level_of_service_int) {
			level_of_service = level_of_service_int;
		}

		// Update time from neighbors on the LDM
		for (int i = 0;i < NAVIGATION_LDM_SIZE;i++) {
			STATUS_MSG_t *smsg = navigation_get_ldm_msg(i);
			if (smsg) {
				if (smsg->id >= 0) {
					float age = (float)chVTTimeElapsedSinceX(smsg->timestamp) / (float)CH_CFG_ST_FREQUENCY;

					if (age < QUAD_DISAPPEAR_TIME && age > ANCH_MAX_UPDATE_DELAY) {
						if ((int)level_of_service > LEVEL_OF_SERVICE_MEDIUM) {
							level_of_service = LEVEL_OF_SERVICE_MEDIUM;
						}
					}
				}
			}
		}

		float g_time = (float)GLOBAL_CLOCK / (float)GLOBAL_CLOCK_FREQUENCY;
		float t_to_period = LOS_PERIOD - fmodf(g_time, LOS_PERIOD);
		static int los_cnt[LEVEL_OF_SERVIICE_NUM];
		static int global_decision_done = 0;
		static int local_decision_done = 0;
		static LEVEL_OF_SERVICE_t los_global_new = LEVEL_OF_SERVICE_HIGH;
		static LEVEL_OF_SERVICE_t los_most_frequent = LEVEL_OF_SERVICE_HIGH;

		if (t_to_period > LOS_BROADCAST_TIME) {
			los_local = level_of_service;
			los_cnt[(int)los_local]++;
			los_global = los_global_new;
			local_decision_done = 0;
		} else {
			if (!local_decision_done) {
				int maximum = los_cnt[0];
				los_cnt[0] = 0;
				los_most_frequent = (LEVEL_OF_SERVICE_t)0;
				for (int i = 1;i < LEVEL_OF_SERVIICE_NUM;i++) {
					if (los_cnt[i] > maximum) {
						maximum  = los_cnt[i];
						los_most_frequent = (LEVEL_OF_SERVICE_t)i;
					}
					los_cnt[i] = 0;
				}
				local_decision_done = 1;
				global_decision_done = 0;
			}

			los_local = los_most_frequent;

			if (t_to_period < LOS_BROADCAST_DECISION_TIME) {
				if (!global_decision_done) {
					los_global_new = los_local;
					if ((int)level_of_service_rec < (int)los_local) {
						los_global_new = level_of_service_rec;
					}
					global_decision_done = 1;
				}
			}
		}

		chThdSleepMilliseconds(20);
	}
}

void safety_update_anchor(ANCHOR_SAFETY_INFO *info) {
	anchor_updates[anchor_update_write] = *info;
	anchor_updates[anchor_update_write].updated = 1;

	anchor_update_write++;
	if (anchor_update_write >= ANCH_UPDATE_LEN) {
		anchor_update_write = 0;
	}
}

void safety_msg_received(SAFETY_MSG_t *msg) {
	LEVEL_OF_SERVICE_t los = LEVEL_OF_SERVICE_HIGH;
	int found = 0;

	for (int i = 0;i < SDM_SIZE;i++) {
		SAFETY_MSG_t *smsg = &sdm[i];

		if (!found && (smsg->id == msg->id || smsg->id <= 0)) {
			*smsg = *msg;
			found = 1;
		}

		if (smsg->id >= 0 && (int)smsg->los_local < (int)los
				&& ((float)chVTTimeElapsedSinceX(smsg->timestamp) /
						(float)CH_CFG_ST_FREQUENCY) < QUAD_DISAPPEAR_TIME) {
			los = smsg->los_local;
		}
	}

	level_of_service_rec = los;
}

void safety_los_int_received(LEVEL_OF_SERVICE_t los) {
	level_of_service_int = los;
}

LEVEL_OF_SERVICE_t safety_get_level_of_service(void) {
	return los_global;
//	return LEVEL_OF_SERVICE_LOW;
}

LEVEL_OF_SERVICE_t safety_get_level_of_service_internal(void) {
	return los_local;
//	return LEVEL_OF_SERVICE_HIGH;
}
