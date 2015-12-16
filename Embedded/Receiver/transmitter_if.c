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

#include "transmitter_if.h"

float transmitter_channel(uint8_t channel) {
	if (channel >= RX_CHANNELS) {
		return 0.0;
	}
#ifdef TRANSMITTER_PPM
	return transmitter_ppm_channel[channel];
#elif defined TRANSMITTER_RECEIVER
	return transmitter_receiver_channel[channel].value;
#endif
}
