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

#ifndef TRANSMITTER_PPM_H_
#define TRANSMITTER_PPM_H_

#include <stdint.h>

#define RECEIVER_TIMER_FREQUENCY	1000000
#define	PPM_CHANNELS				6

#define PPM_LENGTH_MAX	1.6
#define PPM_LENGTH_MIN	0.6

// Global variables
extern float transmitter_ppm_channel[PPM_CHANNELS];

// Functions
void transmitter_ppm_init(void);
void transmitter_ppm_isr(uint8_t pin);
uint32_t transmitter_ppm_get_time_since_update(void);

#endif /* PPM_DECODE_H_ */
