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

#ifndef UTILS_H_
#define UTILS_H_

// Functions
void utils_init(void);
void utils_delay_us(unsigned int us);
float utils_weight_angle(float angle1, float angle2, float ratio);
float utils_angle_difference(float angle1, float angle2);
void utils_norm_angle(float *angle);
int utils_truncate_number(float *number, float min, float max);
float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
void utils_step_towards(float *value, float goal, float step);
void utils_deadband(float *value, float tres, float max);
void utils_sys_lock_cnt(void);
void utils_sys_unlock_cnt(void);

#endif /* UTILS_H_ */
