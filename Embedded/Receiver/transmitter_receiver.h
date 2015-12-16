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

#ifndef TRANSMITTER_RECEIVER_H_
#define TRANSMITTER_RECEIVER_H_

#include <stdint.h>

#define RECEIVER_TIMER_FREQUENCY	1000000
#define RECEIVER_CHANNELS			6

#define PPM_LENGTH_MAX	2.0
#define PPM_LENGTH_MIN	1.0

/*
 * Inputs: PA0, PA1, PB8 and PB9, PC10, PC11
 */

// Type definitions
typedef struct {
	volatile float value;
	volatile uint16_t t_start;
	stm32_gpio_t* port;
	uint8_t pin;
} receiver_channel_t;

// Global variables
extern receiver_channel_t transmitter_receiver_channel[RECEIVER_CHANNELS];

// Functions
void transmitter_receiver_init(void);
void transmitter_receiver_isr(uint8_t pin);
uint32_t transmitter_receiver_get_time_since_update(void);

#endif /* RECEIVER_H_ */
