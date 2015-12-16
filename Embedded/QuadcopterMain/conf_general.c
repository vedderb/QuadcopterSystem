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

#include "conf_general.h"
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "eeprom.h"
#include "utils.h"
#include "packet_handler.h"
#include "stm32f4xx_conf.h"

// EEPROM settings
#define EEPROM_BASE_QUADCONF		1000

// Global variables
QUAD_CONFIG quad_config;
uint16_t VirtAddVarTab[NB_OF_VAR];

void conf_general_init(void) {
	palSetPadMode(GPIOE, 8, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 9, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 10, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 11, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 12, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 13, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 14, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 15, PAL_MODE_INPUT_PULLUP);

	chThdSleepMilliseconds(10);

	// First, make sure that all relevant virtual addresses are assigned for page swapping.
	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	int ind = 0;
	for (unsigned int i = 0;i < (sizeof(QUAD_CONFIG) / 2);i++) {
		VirtAddVarTab[ind++] = EEPROM_BASE_QUADCONF + i;
	}

	utils_sys_lock_cnt();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();
	utils_sys_unlock_cnt();

	memset(&quad_config, 0, sizeof(QUAD_CONFIG));
	conf_general_read_quad_configuration(&quad_config);

	int quad_id = (~(palReadPort(GPIOE) >> 8)) & 0x0F;

	quad_config.quad_id = quad_id;

	switch (quad_config.quad_id) {
	case 0:
		quad_config.mag_cal_cx = -3.82;
		quad_config.mag_cal_cy = -5.15;
		quad_config.mag_cal_cz = -59.1;

		quad_config.mag_cal_xx = 0.757686;
		quad_config.mag_cal_xy = -0.0303832;
		quad_config.mag_cal_xz = -0.0202546;

		quad_config.mag_cal_yx = -0.0303832;
		quad_config.mag_cal_yy = 0.867417;
		quad_config.mag_cal_yz = 0.118851;

		quad_config.mag_cal_zx = -0.0202546;
		quad_config.mag_cal_zy = 0.118851;
		quad_config.mag_cal_zz = 0.883875;
		break;

	case 1:
		quad_config.mag_cal_cx = -3.82;
		quad_config.mag_cal_cy = -5.15;
		quad_config.mag_cal_cz = -59.1;

		quad_config.mag_cal_xx = 0.757686;
		quad_config.mag_cal_xy = -0.0303832;
		quad_config.mag_cal_xz = -0.0202546;

		quad_config.mag_cal_yx = -0.0303832;
		quad_config.mag_cal_yy = 0.867417;
		quad_config.mag_cal_yz = 0.118851;

		quad_config.mag_cal_zx = -0.0202546;
		quad_config.mag_cal_zy = 0.118851;
		quad_config.mag_cal_zz = 0.883875;
		break;

	case 2:
		quad_config.mag_cal_cx = -3.82;
		quad_config.mag_cal_cy = -5.15;
		quad_config.mag_cal_cz = -59.1;

		quad_config.mag_cal_xx = 0.757686;
		quad_config.mag_cal_xy = -0.0303832;
		quad_config.mag_cal_xz = -0.0202546;

		quad_config.mag_cal_yx = -0.0303832;
		quad_config.mag_cal_yy = 0.867417;
		quad_config.mag_cal_yz = 0.118851;

		quad_config.mag_cal_zx = -0.0202546;
		quad_config.mag_cal_zy = 0.118851;
		quad_config.mag_cal_zz = 0.883875;
		break;

	default:
		break;
	}
}

/**
 * Read QUAD_CONFIG from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a QUAD_CONFIG struct to write the read configuration to.
 */
void conf_general_read_quad_configuration(QUAD_CONFIG *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;
	for (unsigned int i = 0;i < (sizeof(QUAD_CONFIG) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_QUADCONF + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	// Set the default configuration
	if (!is_ok) {
		memset(conf, 0, sizeof(QUAD_CONFIG));
		conf->emergency_stop = false;

		conf->anchor_settings[0].id = 0;
		conf->anchor_settings[0].px = 3.0;
		conf->anchor_settings[0].py = -3.0;
		conf->anchor_settings[0].pz = 0.95;

		conf->anchor_settings[1].id = 1;
		conf->anchor_settings[1].px = 0.0;
		conf->anchor_settings[1].py = -3.0;
		conf->anchor_settings[1].pz = 0.95;

		conf->anchor_settings[2].id = 3;
		conf->anchor_settings[2].px = 0.0;
		conf->anchor_settings[2].py = 0.0;
		conf->anchor_settings[2].pz = 0.95;

		conf->anchor_settings[3].id = 4;
		conf->anchor_settings[3].px = 3.0;
		conf->anchor_settings[3].py = 0.0;
		conf->anchor_settings[3].pz = 0.95;

		conf->map_lim.min_x = -0.5;
		conf->map_lim.max_x = 5.0;
		conf->map_lim.min_y = -5.0;
		conf->map_lim.max_y = -0.5;
	}
}

/**
 * Write QUAD_CONFIG to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_quad_configuration(QUAD_CONFIG *conf) {
	utils_sys_lock_cnt();

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(QUAD_CONFIG) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;
		if (EE_WriteVariable(EEPROM_BASE_QUADCONF + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			packet_handler_printf("Flash write error");
			break;
		}
	}

	utils_sys_unlock_cnt();
	return is_ok;
}
