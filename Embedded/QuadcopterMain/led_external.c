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

#include "led_external.h"
#include "ch.h"
#include "hal.h"
#include "ws2811.h"
#include "adconv.h"
#include "packet_handler.h"
#include "safety.h"
#include "control.h"
#include "stm32f4xx_conf.h"

// Settings
#define BATTERY_VOLTAGE_THRESHOLD		10.5

// Private types
typedef enum {
	INDICATION_LOS_NONE = 0,
	INDICATION_LOS_LOW,
	INDICATION_LOS_MEDIUM,
	INDICATION_LOS_HIGH
} INDICATION_LOS;

// Private variables
static THD_WORKING_AREA(led_thread_wa, 1024);
static THD_WORKING_AREA(los_global_thread_wa, 1024);
static THD_WORKING_AREA(los_local_thread_wa, 1024);
static volatile INDICATION_LOS ind_global;
static volatile INDICATION_LOS ind_local;
static volatile bool ind_global_busy;
static volatile bool ind_local_busy;
static volatile LED_EXT_INDICATION_LANDED ind_landed;

// Private function prototypes
static THD_FUNCTION(led_thread, arg);
static THD_FUNCTION(los_global_thread, arg);
static THD_FUNCTION(los_local_thread, arg);
static void stop_and_wait_for_los_threads(void);
static uint32_t scale_color(uint32_t color, float scale);
static void set_front_arm(unsigned int index, uint32_t color);
static void set_back_arm(unsigned int index, uint32_t color);
static void set_front_arm_all(uint32_t color);
static void set_back_arm_all(uint32_t color);
static void set_led_global_los_front(unsigned int index, uint32_t color);
static void set_led_global_los_back(unsigned int index, uint32_t color);
static void set_led_local_los_front(unsigned int index, uint32_t color);
static void set_led_local_los_back(unsigned int index, uint32_t color);
static void set_led_global_los_front_all(uint32_t color);
static void set_led_global_los_back_all(uint32_t color);
static void set_led_local_los_front_all(uint32_t color);
static void set_led_local_los_back_all(uint32_t color);

void led_external_init(void) {
	ind_global = INDICATION_LOS_NONE;
	ind_local = INDICATION_LOS_NONE;
	ind_local_busy = false;
	ind_global_busy = false;
	ind_landed = LED_EXT_INDICATION_LANDED_DIR;

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	RNG_Cmd(ENABLE);

	chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);
	chThdCreateStatic(los_global_thread_wa, sizeof(los_global_thread_wa), NORMALPRIO, los_global_thread, NULL);
	chThdCreateStatic(los_local_thread_wa, sizeof(los_local_thread_wa), NORMALPRIO, los_local_thread, NULL);
}

void led_external_set_landed_indication(LED_EXT_INDICATION_LANDED ind) {
	ind_landed = ind;
}

static THD_FUNCTION(led_thread, arg) {
	(void) arg;
	chRegSetThreadName("LED EXT");

	for(;;) {
		if (packet_handler_get_time_since_stick_update() < 1000) {
			if (control_has_autopilot()) {
				// Note: Only two global LoS are shown.
				switch (safety_get_level_of_service()) {
				case LEVEL_OF_SERVICE_LOW:
				case LEVEL_OF_SERVICE_MEDIUM:
					ind_global = INDICATION_LOS_LOW;
					break;

				case LEVEL_OF_SERVICE_HIGH:
					ind_global = INDICATION_LOS_HIGH;
					break;

				default:
					break;
				}

				switch (safety_get_level_of_service_internal()) {
				case LEVEL_OF_SERVICE_LOW:
					ind_local = INDICATION_LOS_LOW;
					break;

				case LEVEL_OF_SERVICE_MEDIUM:
					ind_local = INDICATION_LOS_MEDIUM;
					break;

				case LEVEL_OF_SERVICE_HIGH:
					ind_local = INDICATION_LOS_HIGH;
					break;

				default:
					break;
				}

				chThdSleepMilliseconds(100);
			} else {
				stop_and_wait_for_los_threads();

				for (int i = 0;i < 6;i++) {
					set_front_arm(i, COLOR_GREEN);
					set_back_arm(5 - i, COLOR_RED);
					set_front_arm(5, COLOR_GREEN);
					set_back_arm(5, COLOR_RED);
					chThdSleepMilliseconds(100);
				}

				float scale = 1.0;

				// Fade all but the first LED to black
				for (int i = 0;i < 50;i++) {
					scale -= 0.02;
					set_front_arm_all(scale_color(COLOR_GREEN, scale));
					set_back_arm_all(scale_color(COLOR_RED, scale));
					set_front_arm(5, COLOR_GREEN);
					set_back_arm(5, COLOR_RED);
					chThdSleepMilliseconds(10);
				}
			}
		} else {
			if (ind_landed == LED_EXT_INDICATION_LANDED_LOS ||
					ind_landed == LED_EXT_INDICATION_LANDED_LOS_DIR) {
				// Note: Only two global LoS are shown.
				switch (safety_get_level_of_service()) {
				case LEVEL_OF_SERVICE_LOW:
				case LEVEL_OF_SERVICE_MEDIUM:
					ind_global = INDICATION_LOS_LOW;
					break;

				case LEVEL_OF_SERVICE_HIGH:
					ind_global = INDICATION_LOS_HIGH;
					break;

				default:
					break;
				}

				switch (safety_get_level_of_service_internal()) {
				case LEVEL_OF_SERVICE_LOW:
					ind_local = INDICATION_LOS_LOW;
					break;

				case LEVEL_OF_SERVICE_MEDIUM:
					ind_local = INDICATION_LOS_MEDIUM;
					break;

				case LEVEL_OF_SERVICE_HIGH:
					ind_local = INDICATION_LOS_HIGH;
					break;

				default:
					break;
				}
				chThdSleepMilliseconds(1000);
			}

			if (ind_landed == LED_EXT_INDICATION_LANDED_LOS_DIR) {
				stop_and_wait_for_los_threads();
				set_front_arm_all(COLOR_BLACK);
				set_back_arm_all(COLOR_BLACK);
				chThdSleepMilliseconds(500);
			}

			if (ind_landed == LED_EXT_INDICATION_LANDED_DIR ||
					ind_landed == LED_EXT_INDICATION_LANDED_LOS_DIR) {
				stop_and_wait_for_los_threads();
				set_front_arm_all(COLOR_BLACK);
				set_back_arm_all(COLOR_BLACK);
				set_front_arm(5, COLOR_GREEN);
				set_back_arm(5, COLOR_RED);
				chThdSleepMilliseconds(1000);
			}

			if (ind_landed == LED_EXT_INDICATION_LANDED_LOS_DIR) {
				set_front_arm_all(COLOR_BLACK);
				set_back_arm_all(COLOR_BLACK);
				chThdSleepMilliseconds(500);
			}
		}

		// Low battery warning
		if (adconv_get_vin() < BATTERY_VOLTAGE_THRESHOLD) {
			stop_and_wait_for_los_threads();

			set_front_arm_all(COLOR_BLACK);
			set_back_arm_all(COLOR_BLACK);
			chThdSleepMilliseconds(500);

			for (int i = 0;i < 3;i++) {
				set_front_arm_all(COLOR_RED);
				set_back_arm_all(COLOR_RED);
				chThdSleepMilliseconds(200);
				set_front_arm_all(COLOR_BLACK);
				set_back_arm_all(COLOR_BLACK);
				chThdSleepMilliseconds(200);
			}
		}
	}
}

static THD_FUNCTION(los_global_thread, arg) {
	(void) arg;
	chRegSetThreadName("LoS Global");

	for(;;) {
		if (ind_global == INDICATION_LOS_NONE) {
			ind_global_busy = false;
			chThdSleepMilliseconds(10);
		} else {
			ind_global_busy = true;

			if (ind_global == INDICATION_LOS_LOW) {
				float scale = 1.0;

				for (int i = 0;i < 25;i++) {
					scale -= 0.04;
					set_led_global_los_front_all(scale_color(COLOR_RED, scale));
					set_led_global_los_back_all(scale_color(COLOR_RED, scale));
					chThdSleepMilliseconds(10);
				}

				for (int i = 0;i < 25;i++) {
					scale += 0.04;
					set_led_global_los_front_all(scale_color(COLOR_RED, scale));
					set_led_global_los_back_all(scale_color(COLOR_RED, scale));
					chThdSleepMilliseconds(10);
				}
			} else if (ind_global == INDICATION_LOS_MEDIUM) {
				float scale = 1.0;

				for (int i = 0;i < 25;i++) {
					scale -= 0.04;
					set_led_global_los_front_all(scale_color(COLOR_ORANGE, scale));
					set_led_global_los_back_all(scale_color(COLOR_ORANGE, scale));
					chThdSleepMilliseconds(10);
				}

				for (int i = 0;i < 25;i++) {
					scale += 0.04;
					set_led_global_los_front_all(scale_color(COLOR_ORANGE, scale));
					set_led_global_los_back_all(scale_color(COLOR_ORANGE, scale));
					chThdSleepMilliseconds(10);
				}
			} else if (ind_global == INDICATION_LOS_HIGH) {
//				for (int i = 0;i < 6;i++) {
//					set_led_global_los_front_all(COLOR_BLACK);
//					set_led_global_los_back_all(COLOR_BLACK);
//					set_led_global_los_front(i, COLOR_GREEN);
//					set_led_global_los_back(i, COLOR_GREEN);
//					if (i < 3) {
//						set_led_global_los_front(i + 3, RNG_GetRandomNumber());
//						set_led_global_los_back(i + 3, RNG_GetRandomNumber());
//					} else {
//						set_led_global_los_front(i - 3, RNG_GetRandomNumber());
//						set_led_global_los_back(i - 3, RNG_GetRandomNumber());
//					}
//					chThdSleepMilliseconds(100);
//				}
				set_led_global_los_front_all(COLOR_GREEN);
				set_led_global_los_back_all(COLOR_GREEN);
				chThdSleepMilliseconds(100);
			}
		}

		chThdSleepMilliseconds(10);
	}
}

static THD_FUNCTION(los_local_thread, arg) {
	(void) arg;
	chRegSetThreadName("LoS Local");

	for(;;) {
		if (ind_local == INDICATION_LOS_NONE) {
			ind_local_busy = false;
			chThdSleepMilliseconds(10);
		} else {
			ind_local_busy = true;

			if (ind_local == INDICATION_LOS_LOW) {
				float scale = 1.0;

				for (int i = 0;i < 25;i++) {
					scale -= 0.04;
					set_led_local_los_front_all(scale_color(COLOR_RED, scale));
					set_led_local_los_back_all(scale_color(COLOR_RED, scale));
					chThdSleepMilliseconds(10);
				}

				for (int i = 0;i < 25;i++) {
					scale += 0.04;
					set_led_local_los_front_all(scale_color(COLOR_RED, scale));
					set_led_local_los_back_all(scale_color(COLOR_RED, scale));
					chThdSleepMilliseconds(10);
				}
			} else if (ind_local == INDICATION_LOS_MEDIUM) {
				float scale = 1.0;

				for (int i = 0;i < 25;i++) {
					scale -= 0.04;
					set_led_local_los_front_all(scale_color(COLOR_ORANGE, scale));
					set_led_local_los_back_all(scale_color(COLOR_ORANGE, scale));
					chThdSleepMilliseconds(10);
				}

				for (int i = 0;i < 25;i++) {
					scale += 0.04;
					set_led_local_los_front_all(scale_color(COLOR_ORANGE, scale));
					set_led_local_los_back_all(scale_color(COLOR_ORANGE, scale));
					chThdSleepMilliseconds(10);
				}
			} else if (ind_local == INDICATION_LOS_HIGH) {
//				for (int i = 0;i < 6;i++) {
//					set_led_local_los_front_all(COLOR_BLACK);
//					set_led_local_los_back_all(COLOR_BLACK);
//					set_led_local_los_front(i, COLOR_GREEN);
//					set_led_local_los_back(i, COLOR_GREEN);
//					if (i < 3) {
//						set_led_local_los_front(i + 3, RNG_GetRandomNumber());
//						set_led_local_los_back(i + 3, RNG_GetRandomNumber());
//					} else {
//						set_led_local_los_front(i - 3, RNG_GetRandomNumber());
//						set_led_local_los_back(i - 3, RNG_GetRandomNumber());
//					}
//					chThdSleepMilliseconds(100);
//				}
				set_led_local_los_front_all(COLOR_GREEN);
				set_led_local_los_back_all(COLOR_GREEN);
				chThdSleepMilliseconds(100);
			}
		}

		chThdSleepMilliseconds(1);
	}
}

static void stop_and_wait_for_los_threads(void) {
	ind_global = INDICATION_LOS_NONE;
	ind_local = INDICATION_LOS_NONE;

	while (ind_global_busy || ind_local_busy) {
		chThdSleepMilliseconds(1);
	}
}

static uint32_t scale_color(uint32_t color, float scale) {
	uint32_t r = (color >> 16) & 0xFF;
	uint32_t g = (color >> 8) & 0xFF;
	uint32_t b = color & 0xFF;

	r *= scale;
	g *= scale;
	b *= scale;

	return (r << 16) | (g << 8) | b;
}

static void set_front_arm(unsigned int index, uint32_t color) {
	if (index < 6) {
		ws2811_set_led_color(index + 12, color);
		ws2811_set_led_color(23 - index, color);
	}
}

static void set_back_arm(unsigned int index, uint32_t color) {
	if (index < 6) {
		ws2811_set_led_color(index, color);
		ws2811_set_led_color(11 - index, color);
	}
}

static void set_front_arm_all(uint32_t color) {
	for (int i = 12;i < 24;i++) {
		ws2811_set_led_color(i, color);
	}
}

static void set_back_arm_all(uint32_t color) {
	for (int i = 0;i < 12;i++) {
		ws2811_set_led_color(i, color);
	}
}

static void set_led_global_los_front(unsigned int index, uint32_t color) {
	switch (index) {
	case 0: ws2811_set_led_color(20, color); break;
	case 1: ws2811_set_led_color(19, color); break;
	case 2: ws2811_set_led_color(18, color); break;
	case 3: ws2811_set_led_color(17, color); break;
	case 4: ws2811_set_led_color(16, color); break;
	case 5: ws2811_set_led_color(15, color); break;
	default: break;
	}
}

static void set_led_global_los_back(unsigned int index, uint32_t color) {
	switch (index) {
	case 0: ws2811_set_led_color(3, color); break;
	case 1: ws2811_set_led_color(4, color); break;
	case 2: ws2811_set_led_color(5, color); break;
	case 3: ws2811_set_led_color(6, color); break;
	case 4: ws2811_set_led_color(7, color); break;
	case 5: ws2811_set_led_color(8, color); break;
	default: break;
	}
}

static void set_led_local_los_front(unsigned int index, uint32_t color) {
	switch (index) {
	case 0: ws2811_set_led_color(23, color); break;
	case 1: ws2811_set_led_color(22, color); break;
	case 2: ws2811_set_led_color(21, color); break;
	case 3: ws2811_set_led_color(14, color); break;
	case 4: ws2811_set_led_color(13, color); break;
	case 5: ws2811_set_led_color(12, color); break;
	default: break;
	}
}

static void set_led_local_los_back(unsigned int index, uint32_t color) {
	switch (index) {
	case 0: ws2811_set_led_color(0, color); break;
	case 1: ws2811_set_led_color(1, color); break;
	case 2: ws2811_set_led_color(2, color); break;
	case 3: ws2811_set_led_color(9, color); break;
	case 4: ws2811_set_led_color(10, color); break;
	case 5: ws2811_set_led_color(11, color); break;
	default: break;
	}
}

static void set_led_global_los_front_all(uint32_t color) {
	for (int i = 0;i < 6;i++) {
		set_led_global_los_front(i, color);
	}
}

static void set_led_global_los_back_all(uint32_t color) {
	for (int i = 0;i < 6;i++) {
		set_led_global_los_back(i, color);
	}
}

static void set_led_local_los_front_all(uint32_t color) {
	for (int i = 0;i < 6;i++) {
		set_led_local_los_front(i, color);
	}
}

static void set_led_local_los_back_all(uint32_t color) {
	for (int i = 0;i < 6;i++) {
		set_led_local_los_back(i, color);
	}
}
