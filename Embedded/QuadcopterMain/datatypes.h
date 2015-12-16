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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include "ch.h"

// Orientation data
typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;
} ATTITUDE_INFO;

typedef struct {
	float min_x;
	float max_x;
	float min_y;
	float max_y;
} MAP_LIMITS_t;

typedef struct {
	int id;
	float px;
	float py;
	float pz;
} ANCHOR_SETTINGS_t;

typedef struct {
	int quad_id;
	bool emergency_stop;

	float mag_cal_cx;
	float mag_cal_cy;
	float mag_cal_cz;

	float mag_cal_xx;
	float mag_cal_xy;
	float mag_cal_xz;

	float mag_cal_yx;
	float mag_cal_yy;
	float mag_cal_yz;

	float mag_cal_zx;
	float mag_cal_zy;
	float mag_cal_zz;

	ANCHOR_SETTINGS_t anchor_settings[MAX_ANCHORS];
	MAP_LIMITS_t map_lim;
} QUAD_CONFIG;

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
	PACKET_CMD_LINE_SEGMENT,
	PACKET_CMD_GET_ULTRA_DIST,
	PACKET_CMD_RADIO_ALIVE,
	PACKET_CMD_SAFETY_MSG,
	PACKET_CMD_STATUS_MSG,
	PACKET_CMD_SET_ANCHORS,
	PACKET_CMD_GET_ANCHORS,
	PACKET_CMD_EMERGENCY_STOP,
	PACKET_CMD_SET_YAW,
	PACKET_CMD_SET_LED_EXT_IND_LANDED
} PACKET_CMD_t;

// Quad-internal commands
typedef enum {
	PACKET_INT_CMD_ALTITUDE = 0,
	PACKET_INT_CMD_RANGE,
	PACKET_INT_CMD_CLOCK,
	PACKET_INT_CMD_LOS
} PACKET_INT_CMD;

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

// Message sent between quads
typedef struct {
	int id;
	systime_t timestamp;
	uint8_t fw_version_major;
	uint8_t fw_version_minor;
	float px;
	float py;
	float pz;
	float roll;
	float pitch;
	float yaw;
	float vx;
	float vy;
	float vbat;
	uint16_t adc_in[4];
} STATUS_MSG_t;

typedef enum {
	ANCHOR_UPDATE_OK = 0,
	ANCHOR_UPDATE_ZERO,
	ANCHOR_UPDATE_BIG_CORR,
	ANCHOR_UPDATE_OUT_OF_POS,
	ANCHOR_UPDATE_SMALL_DT
} ANCHOR_UPDATE_TYPE_t;

typedef struct {
	int id;
	int updated;
	ANCHOR_UPDATE_TYPE_t update_type;
	systime_t update_time;
} ANCHOR_SAFETY_INFO;

#define LEVEL_OF_SERVIICE_NUM		3
typedef enum {
	LEVEL_OF_SERVICE_LOW = 0,
	LEVEL_OF_SERVICE_MEDIUM,
	LEVEL_OF_SERVICE_HIGH
} LEVEL_OF_SERVICE_t;

/*
 * Safety message
 */
typedef struct {
	int id;
	systime_t timestamp;
	LEVEL_OF_SERVICE_t los_local;
	LEVEL_OF_SERVICE_t los_global;
} SAFETY_MSG_t;

// Position state_structure
typedef struct {
	float px;
	float py;
	float pz;
	float vx;
	float vy;
	float acc_roll_err;
	float acc_pitch_err;
	float roll;
	float pitch;
	float yaw;
	systime_t pz_update_time;
} POS_STATE_t;

typedef struct {
	ANCHOR_SETTINGS_t settings;
	systime_t update_time;
	float last_dist;
	float corr_pcxI;
	float corr_pcyI;
	float corr_vcxI;
	float corr_vcyI;
	float corr_acxI;
	float corr_acyI;
	float corr_pcxLast;
	float corr_pcyLast;
	float corr_vcxLast;
	float corr_vcyLast;
	float corr_acxLast;
	float corr_acyLast;
	int wrong_iterations;
	int out_of_pos;
} ANCHOR_STATE_t;

typedef struct {
	volatile float roll_p_gain;
	volatile float roll_i_gain;
	volatile float roll_d_gain_process;
	volatile float roll_d_gain_error;

	volatile float pitch_p_gain;
	volatile float pitch_i_gain;
	volatile float pitch_d_gain_process;
	volatile float pitch_d_gain_error;

	volatile float yaw_p_gain;
	volatile float yaw_i_gain;
	volatile float yaw_d_gain_process;
	volatile float yaw_d_gain_error;
} CONTROL_PARAMETERS_t;

typedef enum {
	LED_EXT_INDICATION_LANDED_DIR = 0,
	LED_EXT_INDICATION_LANDED_LOS,
	LED_EXT_INDICATION_LANDED_LOS_DIR,
	LED_EXT_INDICATION_LANDED_BATTERY
} LED_EXT_INDICATION_LANDED;

#endif /* DATATYPES_H_ */
