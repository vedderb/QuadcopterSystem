/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se
	Copyright 2013-2014 Daniel Skarin	daniel.skarin@sp.se

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

#ifndef UDP_COMMANDS_H_
#define UDP_COMMANDS_H_

typedef enum {
	CMD_CLIENT_SEND_PULSE_SAMPLE = 100,
	CMD_CLIENT_SEND_PULSE_SAMPLE_ALLDATA,
	CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST,
	CMD_CLIENT_SET_LOS
} CMD_CLIENT_t;

typedef enum {
	  CMD_SERVER_TEXT  = 0,
	    CMD_SERVER_LOGGED_PULSE,
	    CMD_SERVER_LOGGED_CORRELATION,
	    CMD_SERVER_LOGGED_PULSE_PART,
	    CMD_SERVER_LOGGED_CORRELATION_PART,
	    CMD_SERVER_DISTANCES,
	    CMD_SERVER_QUAD_PACKET
} CMD_SERVER_t;



// Packet commands
typedef enum {
	PACKET_CMD_SET_STICK_INPUT = 0,
	PACKET_CMD_GET_CONTROL_PARAMETERS,
	PACKET_CMD_SET_CONTROL_PARAMETERS,
	PACKET_CMD_GET_ORIENTATION,
	PACKET_CMD_RESET_GYRO_STICKS,
	PACKET_CMD_RESET_ORIENTATION,
	PACKET_CMD_GET_RAW_IMU,
	PACKET_CMD_OVERRIDE_POWER,
	PACKET_CMD_PRINT,
	PACKET_CMD_GET_POS,
	PACKET_CMD_SET_POS,
	PACKET_CMD_GET_ALTITUDE,
	PACKET_CMD_MAP,
	PACKET_CMD_LDM_ELEMENT,
	PACKET_CMD_LINE_SEGMENT
} PACKET_CMD_t;

// The first POS_MASK_BITS bits are the type, the rest of the bits
// are attributes for the type
typedef enum {
    POS_TYPE_NORMAL = 0,
    POS_TYPE_COLLISION
} POS_TYPE_t;

typedef enum {
    POS_MASK_TYPE = 0xF,
    POS_MASK_BITS = 4,
    // Normal
    POS_MASK_PERCEIVED = (1 << (POS_MASK_BITS + 0)),
    POS_MASK_FOLLOWING = (1 << (POS_MASK_BITS + 1)),
    POS_MASK_HAS_COLLISION = (1 << (POS_MASK_BITS + 2))
    // Collision
} POS_MASK_t;


// Settings
#define PWM_RX_CH_NUM			6

// Channel mapping
#define CH_THROTTLE				3
#define CH_ROLL					0
#define CH_PITCH				1
#define CH_YAW					2
#define CH_POT_L				4
#define CH_POT_R				5

// Limits
#define LIM_THROTTLE_MIN		0.13
#define LIM_THROTTLE_MAX		0.88
#define LIM_ROLL_MIN			0.11
#define LIM_ROLL_MAX			0.90
#define LIM_PITCH_MIN			0.11
#define LIM_PITCH_MAX			0.88
#define LIM_YAW_MIN				0.13
#define LIM_YAW_MAX				0.93
#define LIM_POT_L_MIN			0.0
#define LIM_POT_L_MAX			1.0
#define LIM_POT_R_MIN			0.0
#define LIM_POT_R_MAX			1.0

// Inversion
#define INV_THROTTLE			0
#define INV_ROLL				0
#define INV_PITCH				0
#define INV_YAW					0
#define INV_POT_L				0
#define INV_POT_R				0

// Default center values (may be overridden after calibration)
#define CENTER_ROLL				0.5
#define CENTER_PITCH			0.5
#define CENTER_YAW				0.5


#endif /* UDP_COMMANDS_H_ */
