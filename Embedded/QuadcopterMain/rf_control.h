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

// Settings
#define RF_CHANNEL				12 // 2.4 GHz RF channel
#define PAN_ID					0xfa11
#define NODE_ADDRESS			0x001
#define DEST_ADDRESS			0xffff

// Functions
void rf_control_init(void);
void rf_control_send_packet(uint8_t *data, uint8_t len);

#endif /* RF_CONTROL_H_ */
