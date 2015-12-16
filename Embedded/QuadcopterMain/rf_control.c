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
#include "rf_control.h"
#include "packet_handler.h"

// CC2520
#include "hal_cc2520.h"
#include "hal_rf.h"
#include "hal_rf_util.h"
#include "basic_rf.h"

#include <string.h>

// Settings
#define RX_BUFFER_LENGTH	255
#define TX_BUFFER_LENGTH	255
#define TX_BUFFER_SLOTS		10
#define TX_DELAY_US			5000

// Private variables
static basicRfCfg_t basicRfConfig;
static uint8_t rx_buffer[RX_BUFFER_LENGTH];
static THD_WORKING_AREA(rf_control_thread_wa, 2048);
static THD_WORKING_AREA(rf_tx_thread_wa, 512);
static uint8_t tx_buffer[TX_BUFFER_SLOTS][TX_BUFFER_LENGTH];
static uint8_t tx_slot_len[TX_BUFFER_SLOTS];
static int tx_slot_read;
static int tx_slot_write;
static thread_t *tx_tp;
static virtual_timer_t vt;

#ifdef SECURITY_CCM
static uint8_t rf_security_key[] = {
		45, 22, 67, 1,
		56, 2, 32, 233,
		90, 97, 54, 44,
		59, 131, 37, 67
};
#endif

// Private functions
static THD_FUNCTION(rf_control_thread, arg);
static THD_FUNCTION(rf_tx_thread, arg);
static void wakeup_tx(void *p);

void rf_control_init(void) {
	tx_slot_read = 0;
	tx_slot_write = 0;

	// rf
	halAssyInit();
	basicRfConfig.panId = PAN_ID;
	basicRfConfig.channel = RF_CHANNEL;
	basicRfConfig.ackRequest = FALSE;
	basicRfConfig.myAddr = NODE_ADDRESS;
#ifdef SECURITY_CCM
	basicRfConfig.securityKey = rf_security_key;
#endif

	if(basicRfInit(&basicRfConfig) == FAILED) {
		for(;;) {}
	}

	basicRfReceiveOn();

	chThdCreateStatic(rf_control_thread_wa, sizeof(rf_control_thread_wa),
			NORMALPRIO, rf_control_thread, NULL);
	chThdCreateStatic(rf_tx_thread_wa, sizeof(rf_tx_thread_wa),
			NORMALPRIO, rf_tx_thread, NULL);
}

/*
 * Note that this packet is stored and sent in another thread after a delay. The delay is done
 * in case that this packet is a request from the (Qt) client, which means that more requests
 * can come within a short time. We want to give the additional requests a chance to arrive
 * before blocking the RF channel by sending the response.
 */
void rf_control_send_packet(uint8_t *data, uint8_t len) {
	memcpy(tx_buffer[tx_slot_write], data, len);
	tx_slot_len[tx_slot_write] = len;

	tx_slot_write++;
	if (tx_slot_write >= TX_BUFFER_SLOTS) {
		tx_slot_write = 0;
	}

	chSysLock();
	if (!chVTIsArmedI(&vt)) {
		chVTSetI(&vt, US2ST(TX_DELAY_US), wakeup_tx, NULL);
	}
	chSysUnlock();
}

static THD_FUNCTION(rf_control_thread, arg) {
	(void)arg;
	uint8_t len;

	chRegSetThreadName("RF Control");

	for(;;) {
		if (basicRfPacketIsReady()) {
			len = basicRfReceive(rx_buffer, RX_BUFFER_LENGTH, NULL);
			packet_handler_process_packet(rx_buffer, len);
		}

		chThdSleepMicroseconds(100);
	}
}

static THD_FUNCTION(rf_tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("RF Tx");

	tx_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while(tx_slot_read != tx_slot_write) {
			basicRfSendPacket(DEST_ADDRESS, tx_buffer[tx_slot_read], tx_slot_len[tx_slot_read]);

			tx_slot_read++;
			if (tx_slot_read >= TX_BUFFER_SLOTS) {
				tx_slot_read = 0;
			}
		}

		basicRfReceiveOn();
	}
}

static void wakeup_tx(void *p) {
	(void)p;

	chSysLockFromISR();
	chEvtSignalI(tx_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}
