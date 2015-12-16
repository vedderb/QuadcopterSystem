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

#ifndef TRANSMITTER_IF_H_
#define TRANSMITTER_IF_H_

#define	TRANSMITTER_PPM
//#define TRANSMITTER_RECEIVER

#include "ch.h"
#include "hal.h"
#include <stm32f4xx.h>

#ifdef TRANSMITTER_PPM
#include "transmitter_ppm.h"
#define RX_CHANNELS		PPM_CHANNELS
#elif defined TRANSMITTER_RECEIVER
#include "transmitter_receiver.h"
#define RX_CHANNELS		RECEIVER_CHANNELS
#endif

// Functions
float transmitter_channel(uint8_t channel);
#ifdef TRANSMITTER_PPM
#define transmitter_init() transmitter_ppm_init()
#define transmitter_isr(x)	transmitter_ppm_isr(x)
#define transmitter_get_time_since_update() transmitter_ppm_get_time_since_update()
#elif defined TRANSMITTER_RECEIVER
#define transmitter_init() transmitter_receiver_init()
#define transmitter_isr(x)	transmitter_receiver_isr(x)
#define transmitter_get_time_since_update() transmitter_receiver_get_time_since_update()
#endif

#endif /* TRANSMITTER_IF_H_ */
