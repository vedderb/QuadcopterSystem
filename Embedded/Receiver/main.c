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

// Set to enable reading of PPM or PWM signals
//#define ENABLE_PPM_OR_PWM

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "comm_usb_serial.h"
#include "ext_cb.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "rf_control.h"

// RF
#include "hal_cc2520.h"
#include "hal_rf.h"
#include "hal_rf_util.h"
#include "basic_rf.h"


#include "packet.h"
#include "led.h"


#ifdef ENABLE_PPM_OR_PWM
#include "transmitter_if.h"
#endif

/*
 * Timers used:
 * TIM3: transmitter_if
 */

// Settings
#define RX_BUFFER_LENGTH 128

// Private variables
static basicRfCfg_t basicRfConfig;
static THD_WORKING_AREA(serial_read_thread_wa, 1024);
static THD_WORKING_AREA(serial_send_thread_wa, 1024);
static THD_WORKING_AREA(rf_read_thread_wa, 1024);
static THD_WORKING_AREA(timer_thread_wa, 128);

#define SERIAL_RX_BUFFER_SIZE		4096
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

#ifdef SECURITY_CCM
static uint8_t rf_security_key[] = {
		45, 22, 67, 1,
		56, 2, 32, 233,
		90, 97, 54, 44,
		59, 131, 37, 67
};
#endif

// Private functions
static void process_packet(unsigned char *buffer, unsigned char len);
static void send_packet(unsigned char *buffer, unsigned char len);

static THD_FUNCTION(serial_read_thread, arg) {
	(void)arg;

	uint8_t buffer[128];
	int i;
	int len;

	for(;;) {
		len = chSequentialStreamRead(&SDU1, (uint8_t*) buffer, 1);

		for (i = 0;i < len;i++) {
			serial_rx_buffer[serial_rx_write_pos++] = buffer[i];

			if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_write_pos = 0;
			}
		}
	}
}

static THD_FUNCTION(serial_send_thread, arg) {
	(void)arg;

	for(;;) {
		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], 0);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}

		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(rf_read_thread, arg) {
	(void)arg;

	uint8_t buffer[200];
	int len;

	for(;;) {
		if (basicRfPacketIsReady()) {
			led_write(LED_RED, 1);
			len = basicRfReceive(buffer, RX_BUFFER_LENGTH, NULL);
			packet_send_packet(buffer, len, 0);
			led_write(LED_RED, 0);
		}

		chThdSleepMicroseconds(100);
	}
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	for(;;) {
		packet_timerfunc();
		chThdSleepMilliseconds(1);
	}
}

static void process_packet(unsigned char *buffer, unsigned char len) {
	if (!len) {
		return;
	}

	basicRfSendPacket(0xffff, (uint8 *)buffer, len);
}

static void send_packet(unsigned char *buffer, unsigned char len) {
	chSequentialStreamWrite(&SDU1, buffer, len);
}

int main(void) {
	halInit();
	chSysInit();
	led_init();
	ext_cb_init();
	comm_usb_serial_init();

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
		for(;;) {
		}
	}
	basicRfReceiveOn();

#ifdef ENABLE_PPM_OR_PWM
	transmitter_init();
#endif
	rf_control_init();

	// Packets
	packet_init(send_packet, process_packet, 0);

	// Threads
	chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa), NORMALPRIO, serial_read_thread, NULL);
	chThdCreateStatic(serial_send_thread_wa, sizeof(serial_send_thread_wa), NORMALPRIO, serial_send_thread, NULL);
	chThdCreateStatic(rf_read_thread_wa, sizeof(rf_read_thread_wa), NORMALPRIO, rf_read_thread, NULL);
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

	while (TRUE) {
		chThdSleepMilliseconds(100);
	}
}
