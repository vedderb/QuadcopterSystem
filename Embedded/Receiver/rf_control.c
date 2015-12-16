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

#include "ch.h"
#include "hal.h"
#include "comm_usb_serial.h"
#include "chprintf.h"
#include <stdlib.h>
#include "basic_rf.h"
#include "rf_control.h"
#include "transmitter_if.h"

// Private variables
#define RX_BUFFER_LENGTH 128
uint8_t rx_buffer[RX_BUFFER_LENGTH];
static THD_WORKING_AREA(rf_thread_wa, 512);

// Private functions
static THD_FUNCTION(rf_thread, arg);

void rf_control_init(void) {
	chThdCreateStatic(rf_thread_wa, sizeof(rf_thread_wa), NORMALPRIO, rf_thread, NULL);
}

static THD_FUNCTION(rf_thread, arg) {
	(void)arg;
	uint8_t tx_buf[2 * RX_CHANNELS + 2];
	float v;
	chRegSetThreadName("RF Control");

	chThdSleepMilliseconds(1500);

	while(1) {
		if (transmitter_get_time_since_update() < 1000) {
			uint8_t id = 0;

			if (transmitter_channel(CH_POT_R) < 0.15) {
				id = 0;
			} else if (transmitter_channel(CH_POT_R) > 0.85) {
				id = 1;
			} else {
				id = 35;
			}

			tx_buf[0] = id;
			tx_buf[1] = PACKET_CMD_SET_STICK_INPUT;
			for (int i = 0; i < RX_CHANNELS; i++) {
				v = transmitter_channel(i);

				if (i == CH_ROLL || i == CH_PITCH) {
					v = 1.0 - v;
				}

				tx_buf[2 * i + 2] = (uint8_t)i;
				tx_buf[2 * i + 3] = (uint8_t) (v*255);
#ifdef DEBUG
				if (i == 0) {
					chprintf((BaseSequentialStream *)&SDU1, "\r\n");
				}
				chprintf((BaseSequentialStream *)&SDU1, "Channel %d: %.2f\r\n", i, v);
#endif
			}

			basicRfSendPacket(0xffff, (uint8 *) tx_buf, 2 * RX_CHANNELS + 2);

			// Send alive packet to all quads
//			tx_buf[0] = QUAD_ID_ALL;
//			tx_buf[1] = PACKET_CMD_RADIO_ALIVE;
//			basicRfSendPacket(0xffff, (uint8 *)tx_buf, 2);
		}

#ifdef DEBUG
		chThdSleepMilliseconds(100);
#else
		chThdSleepMilliseconds(10);
#endif
	}
}

