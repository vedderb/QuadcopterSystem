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

#include "ext_cb.h"
#include "ch.h"
#include "hal.h"
#include "hal_rf.h"
#include "transmitter_if.h"

static void transmitter_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	transmitter_isr(channel);
}

static const EXTConfig extcfg = {
		{
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA, transmitter_cb},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOA, transmitter_cb},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOC, transmitter_cb},
				{EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, halRfExtCb},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB, transmitter_cb},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOB, transmitter_cb},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOC, transmitter_cb},
				{EXT_CH_MODE_BOTH_EDGES | EXT_MODE_GPIOC, transmitter_cb},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL},
				{EXT_CH_MODE_DISABLED, NULL}
		}
};

void ext_cb_init(void) {
	extStart(&EXTD1, &extcfg);
}
