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

#include "broadcast.h"
#include "safety.h"
#include "ch.h"
#include "led.h"
#include "packet_handler.h"
#include "packet_handler_int.h"
#include "pos.h"
#include "adconv.h"
#include <string.h>

// Settings
#define BROADCAST_PERIOD		40

static THD_WORKING_AREA(broadcast_thread_wa, 2048);

// Private functions
static THD_FUNCTION(broadcast_thread, arg);

void broadcast_init(void) {
	chThdCreateStatic(broadcast_thread_wa, sizeof(broadcast_thread_wa), NORMALPRIO, broadcast_thread, NULL);
}

static THD_FUNCTION(broadcast_thread, arg) {
	(void) arg;
	chRegSetThreadName("Broadcast");

	for (;;) {
		uint32_t g_time = GLOBAL_CLOCK / (GLOBAL_CLOCK_FREQUENCY / CH_CFG_ST_FREQUENCY);
		systime_t delay = MS2ST(BROADCAST_PERIOD) - (g_time % MS2ST(BROADCAST_PERIOD));

		// MS2ST with parameter 0 does not work
		if (quad_config.quad_id == 0) {
			chThdSleep(delay);
		} else {
			chThdSleep(delay + MS2ST(5 * quad_config.quad_id));
		}

		SAFETY_MSG_t safety_msg;
		safety_msg.id = quad_config.quad_id;
		safety_msg.los_local = safety_get_level_of_service_internal();
		safety_msg.los_global = safety_get_level_of_service();
		packet_handler_send_safety_msg(QUAD_ID_ALL, &safety_msg);

		STATUS_MSG_t status_msg;
		POS_STATE_t *pos = pos_get_state();
		status_msg.id = quad_config.quad_id;
		status_msg.fw_version_major = FW_VERSION_MAJOR;
		status_msg.fw_version_minor = FW_VERSION_MINOR;
		status_msg.px = pos->px;
		status_msg.py = pos->py;
		status_msg.pz = pos->pz;
		status_msg.roll = pos->roll;
		status_msg.pitch = pos->pitch;
		status_msg.yaw = pos->yaw;
		status_msg.vx = pos->vx;
		status_msg.vy = pos->vy;
		status_msg.vbat = adconv_get_vin();
		for (int i = 0;i < 4;i++) {
			status_msg.adc_in[i] = adconv_get_adc_pin(i);
		};
		packet_handler_send_status_msg(QUAD_ID_ALL, &status_msg);
	}
}
