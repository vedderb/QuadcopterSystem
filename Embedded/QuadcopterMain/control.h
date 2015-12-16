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

#ifndef CONTROL_H_
#define CONTROL_H_

// Functions
void control_init(void);
CONTROL_PARAMETERS_t *control_get_pid_parameters(void);
void control_get_orientation_now(float *roll, float *pitch, float *yaw);
void control_reset_orientation_offsets(void);
void control_set_yaw(float angle);
void control_override_power(int motor, float power);
float control_get_last_iteration_delay_ms(void);
void control_get_accel_gyro_mag_used(float *accel, float *gyro, float *mag);
void control_update_initial_att(void);
int control_has_autopilot(void);

#endif /* CONTROL_H_ */
