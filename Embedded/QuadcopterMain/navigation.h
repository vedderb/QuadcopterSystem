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

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "packet_handler.h"
#include "conf_general.h"

//Settings
#define NAVIGATION_LDM_SIZE				10

// Functions
void navigation_init(void);
void navigation_override_xy(float x_pos, float y_pos);
void navigation_override_xy_stop(void);
void navigation_override_z(float z_pos);
void navigation_override_z_stop(void);
void navigation_run_xy(float *roll_out, float *pitch_out, float dt);
void navigation_run_z(float *throttle_out);
void navigation_send_set_pos_to_map(void);
void navigation_status_received(STATUS_MSG_t *msg);
STATUS_MSG_t *navigation_get_ldm_msg(int id);

#endif /* NAVIGATION_H_ */
