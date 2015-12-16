/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se

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
 
#ifndef COMM_H_
#define COMM_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PACKET_INT_CMD_ALTITUDE = 0,
	PACKET_INT_CMD_RANGE,
	PACKET_INT_CMD_CLOCK,
	PACKET_INT_CMD_LOS
} PACKET_INT_CMD;

#define COMM_USE_CAN		1

// Functions
void comm_init(void);
void comm_sendmsg(uint8_t id, uint8_t *data, uint8_t len);
int comm_is_tx_busy(void);

#endif /* COMM_H_ */
