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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// Software version
#define FW_VERSION_MAJOR	2
#define FW_VERSION_MINOR	0

// General settings
#define QUAD_ID_ALL			255
#define MAG_COMPENSATE		1 // Should be 0 when capturing samples for the calibration

// POS Settings
#define POS_MAX_V			3.0
#define POS_REG_MARGIN		0.5
#define POS_VEL_DECAY_E		0.8
#define POS_VEL_DECAY_L		0.02
#define MAX_ANCHORS 		4
#define MIN_ALTUTUDE		0.05
#define MAX_ALTUTUDE		5.0
#define QUAD_DISAPPEAR_TIME	30.0
#define EMERGENCY_STOP_TIME	2.0

/*
 * Actuator settings
 */

// Which motors are turning clockwise or counterclockwise
#define CW_MODE				1

// Fly as X (0) or + (1)
#define ORIENTATION_MODE	1

/*
 * Motor mapping (X mode)
 *
 * CW_MODE 0
 * FL: CCW
 * BL: CW
 * FR: CW
 * BR: CCW
 *
 * CW_MODE 1
 * FL: CW
 * BL: CCW
 * FR: CCW
 * BR: CW
 */
#define MOTOR_FL			1
#define MOTOR_BL			3
#define MOTOR_FR			2
#define MOTOR_BR			0

/*
 * Motor mapping (+ mode)
 *
 * CW_MODE 0
 * F: CCW
 * L: CW
 * R: CW
 * B: CCW
 *
 * CW_MODE 1
 * F: CWquad_config
 * L: CCW
 * R: CCW
 * B: CW
 */
#define MOTOR_F				0
#define MOTOR_L				1
#define MOTOR_R				2
#define MOTOR_B				3

// Includes
#include <stdbool.h>
#include "datatypes.h"

// Global variables
extern QUAD_CONFIG quad_config;

// Functions
void conf_general_init(void);
void conf_general_read_quad_configuration(QUAD_CONFIG *conf);
bool conf_general_store_quad_configuration(QUAD_CONFIG *conf);

#endif /* CONF_GENERAL_H_ */
