/*
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

#ifndef RF_CONTROL_H_
#define RF_CONTROL_H_

#define RF_CHANNEL				12
#define PAN_ID					0xfa11
#define NODE_ADDRESS			0x003

// Settings
#define QUAD_ID_ALL				255

// Channel mapping
#define CH_THROTTLE				3
#define CH_ROLL					0
#define CH_PITCH				1
#define CH_YAW					2
#define CH_POT_L				4
#define CH_POT_R				5

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
	PACKET_CMD_RADIO_ALIVE
} PACKET_CMD_t;

// Functions
void rf_control_init(void);

#endif /* RF_CONTROL_H_ */
