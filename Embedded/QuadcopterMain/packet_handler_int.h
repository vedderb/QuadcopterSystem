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

#ifndef PACKET_HANDLER_INT_H_
#define PACKET_HANDLER_INT_H_

#include <stdint.h>

#define GLOBAL_CLOCK			(TIM5->CNT)
#define GLOBAL_CLOCK_FREQUENCY	1000000

// Functions
void packet_handler_int_init(void);
void packet_handler_int_send_packet(uint8_t *data, uint8_t len);
float packet_handler_int_get_last_altitude(void);
uint32_t packet_handler_int_get_time_since_update_altitude(void);
systime_t packet_handler_int_get_altitude_timestamp(void);
void packet_handler_int_process_packet(unsigned char *buffer, unsigned char len);

#endif /* PACKET_HANDLER_INT_H_ */
