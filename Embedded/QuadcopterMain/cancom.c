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

#include "cancom.h"
#include "ch.h"
#include "hal.h"
#include "led.h"
#include "packet_handler_int.h"
#include "stm32f4xx_conf.h"

// Settings
#define CANDx		CAND1

// Threads
static THD_WORKING_AREA(cancom_thread_wa, 2048);
static THD_FUNCTION(cancom_thread, arg);

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
		CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};

void cancom_init(void) {
	palSetPadMode(GPIOD, 0,
			PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOD, 1,
			PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);

	canStart(&CANDx, &cancfg);

	chThdCreateStatic(cancom_thread_wa, sizeof(cancom_thread_wa), NORMALPRIO + 1,
			cancom_thread, NULL);
}

static THD_FUNCTION(cancom_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN");

	event_listener_t el;
	CANRxFrame rxmsg;
	uint8_t buffer[9];

	chEvtRegister(&CANDx.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0) {
			continue;
		}

		while (canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
			buffer[0] = rxmsg.SID;
			for (int i = 0;i < rxmsg.DLC;i++) {
				buffer[i + 1] = rxmsg.data8[i];
			}
			packet_handler_int_process_packet(buffer, rxmsg.DLC + 1);
		}
	}

	chEvtUnregister(&CAND1.rxfull_event, &el);
}

void cancom_transmit(uint32_t id, uint8_t *data, uint8_t len) {
	CANTxFrame txmsg;

	txmsg.IDE = CAN_IDE_STD;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;

	for (int i = 0;i < len;i++) {
		txmsg.data8[i] = data[i];
	}

	canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}
