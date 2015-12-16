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

#include "actuator.h"
#include "pwm_esc.h"
#include "utils.h"
#include "conf_general.h"
#include "adconv.h"
#include <stdint.h>

// Private variables
static volatile int orientation_mode = ORIENTATION_MODE;

/*
 * Throttle range: [0.0, 1.0]
 * Roll range:    [-1.0, 1.0]
 * Pitch range:   [-1.0, 1.0]
 * Yaw range:     [-1.0, 1.0]
 */
void actuator_set_output(float throttle, float roll, float pitch, float yaw) {
	float motors[4];
	int i;

#if CW_MODE == 0
	if (orientation_mode == 0) {
		motors[MOTOR_FL] = throttle + roll + pitch - yaw;
		motors[MOTOR_BL] = throttle + roll - pitch + yaw;
		motors[MOTOR_FR] = throttle - roll + pitch + yaw;
		motors[MOTOR_BR] = throttle - roll - pitch - yaw;
	} else {
		motors[MOTOR_F] = throttle + pitch - yaw;
		motors[MOTOR_L] = throttle + roll + yaw;
		motors[MOTOR_R] = throttle - roll + yaw;
		motors[MOTOR_B] = throttle - pitch - yaw;
	}
#else
	if (orientation_mode == 0) {
		motors[MOTOR_FL] = throttle + roll + pitch + yaw;
		motors[MOTOR_BL] = throttle + roll - pitch - yaw;
		motors[MOTOR_FR] = throttle - roll + pitch - yaw;
		motors[MOTOR_BR] = throttle - roll - pitch + yaw;
	} else {
		motors[MOTOR_F] = throttle + pitch + yaw;
		motors[MOTOR_L] = throttle + roll - yaw;
		motors[MOTOR_R] = throttle - roll - yaw;
		motors[MOTOR_B] = throttle - pitch + yaw;
	}
#endif

	for (i = 0;i < 4;i++) {
		// Charge-based throttle scaling
		motors[i] *= (10.5 / adconv_get_vin());

		utils_truncate_number(&motors[i], 0.0, 1.0);
		pwm_esc_set(i, (uint8_t)(motors[i] * 255.0));
	}
}

void actuator_set_orientation_mode(int mode) {
	orientation_mode = mode;
}
