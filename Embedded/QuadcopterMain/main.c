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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "comm_usb_serial.h"
#include "stm32f4xx_conf.h"
#include "shell.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "pwm_esc.h"
#include "rf_control.h"
#include "utils.h"
#include "mpu9150.h"
#include "control.h"
#include "led.h"
#include "packet_handler.h"
#include "basic_rf.h"
#include "pos.h"
#include "ext_cb.h"
#include "packet_handler_int.h"
#include "conf_general.h"
#include "safety.h"
#include "navigation.h"
#include "cancom.h"
#include "adconv.h"
#include "broadcast.h"
#include "ws2811.h"
#include "led_external.h"

/*
 * Timers used:
 * TIM13: utils
 * TIM6: Control
 * TIM3: PPM
 * TIM5: Global clock
 *
 * DMA/Stream	Device		Usage
 * 2, 4			ADC1		Battery voltage measurement
 * 1, 0			I2C1		MPU9150
 * 1, 6			I2C1		MPU9150
 * 1, 3			SPI2		CC2520
 * 1, 4			SPI2		CC2520
 * 1, 7			TIM4		WS2811 LEDs
 *
 */

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)

static void cmd_esc(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc != 2) {
		chprintf(chp, "Usage:\r\n   esc id value\r\n   id: 0-4\r\n   value: 0-255\r\n");
		return;
	}
	chprintf(chp, "Setting duty cycle to %d for id %d\r\n", atoi(argv[1]), atoi(argv[0]));
	pwm_esc_set(atoi(argv[0]), atoi(argv[1]));
}

static void cmd_esc_all(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc != 1) {
		chprintf(chp, "Usage: esca 0-255\r\n");
		return;
	}
	chprintf(chp, "Setting duty cycle to %d for all channels\r\n", atoi(argv[0]));
	pwm_esc_set_all(atoi(argv[0]));
}

static void cmd_rf(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;

	uint8_t tx_buf[80];

	if (argc == 0) {
		chprintf(chp, "Usage: rf b0 b1 ...\r\n");
		return;
	}

	chprintf(chp, "Data:");
	for (int i = 0; i < argc; i++) {
		tx_buf[i] = atoi(argv[i]);
		chprintf(chp, " 0x%X", tx_buf[i]);
	}
	chprintf(chp, "\r\n");

	basicRfSendPacket(0xffff, (uint8* ) tx_buf, argc);
	basicRfReceiveOn();
}

static const ShellCommand commands[] = {
		{"esc", cmd_esc},
		{"esca", cmd_esc_all},
		{"rf", cmd_rf},
		{"mpu", mpu9150_cmd_print},
		{"mpuofs", mpu9150_cmd_sample_offsets},
		{"pktchs", packet_handler_cmd_print},
		{"pktofs", packet_handler_cmd_sample_centers},
		{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream *)&SDU1,
		commands
};

int main(void) {
	halInit();
	chSysInit();
	conf_general_init();
	adconv_init();
	utils_init();
	safety_init();
	comm_usb_serial_init();
	led_init();
	ext_cb_init();
	shellInit();
	pwm_esc_init();
	pos_init();

	// Internal communications
	packet_handler_int_init();

	// Packet control
	// NOTE: Has to be after the internal communication init
	// because of the DMA overlap.
	packet_handler_init();
	rf_control_init();

	// Sensors
	mpu9150_init();

	chThdSleepMilliseconds(1000);
	led_write(LED_RED, 1);
	mpu9150_sample_gyro_offsets(100);
	led_write(LED_RED, 0);

	// Control
	control_init();

	// Navigation
	navigation_init();

	// CAN
	cancom_init();

	// Message broadcasting
	broadcast_init();

	// WS2811 LEDs (PD14, Servo7, P12)
	ws2811_init();
	led_external_init();

	// Initial position
	chThdSleepMilliseconds(100);
	POS_STATE_t *pos_state = pos_get_state();
	pos_state->px = 1.0;
	pos_state->py = -1.0;
	pos_state->vx = 0.0;
	pos_state->vy = 0.0;
	control_set_yaw(0.0);
	pos_state->yaw = 0.0;

	static thread_t *shelltp = NULL;

	while (TRUE) {
		if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminatedX(shelltp)) {
			chThdRelease(shelltp);
			shelltp = NULL;
		}

		chThdSleepMilliseconds(100);
	}
}
