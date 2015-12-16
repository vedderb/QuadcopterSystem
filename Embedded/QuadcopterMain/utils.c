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

#include "utils.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <math.h>

// Private variables
static volatile int sys_lock_cnt = 0;

void utils_init(void) {
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	// Microsecond delay timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	uint16_t PrescalerValue = (uint16_t) ((RCC_Clocks.SYSCLK_Frequency / 2) / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM13, ENABLE);
}

void utils_delay_us(unsigned int us) {
	unsigned int goal = TIM13->CNT;
	goal += us;
	goal &= 0xFFFF;

	while(TIM13->CNT > goal){};
	while(TIM13->CNT < goal){};
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2) {
	utils_norm_angle(&angle1);
	utils_norm_angle(&angle2);

	if (fabsf(angle1 - angle2) > 180.0) {
		if (angle1 < angle2) {
			angle1 += 360.0;
		} else {
			angle2 += 360.0;
		}
	}

	return angle1 - angle2;
}

/**
 * Run a "complementary filter" on two angles
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @param ratio
 * The ratio between the first and second angle.
 * @return
 */
float utils_weight_angle(float angle1, float angle2, float ratio) {
	utils_norm_angle(&angle1);
	utils_norm_angle(&angle2);

	if (fabsf(angle1 - angle2) > 180.0) {
		if (angle1 < angle2) {
			angle1 += 360.0;
		} else {
			angle2 += 360.0;
		}
	}

	float angle_weighted = angle1 * ratio + angle2 * (1 - ratio);
	utils_norm_angle(&angle_weighted);

	return angle_weighted;
}

/**
 * Make sure that 0 <= angle < 360
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0);

	if (*angle < 0.0) {
		*angle += 360.0;
	}
}

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void utils_step_towards(float *value, float goal, float step) {
	if (*value < goal) {
		if ((*value + step) < goal) {
			*value += step;
		} else {
			*value = goal;
		}
	} else if (*value > goal) {
		if ((*value - step) > goal) {
			*value -= step;
		} else {
			*value = goal;
		}
	}
}

/**
 * Truncate absolute values less than tres to zero. If value is more than
 * tres, tres will be mapped to 0 and max to max.
 */
void utils_deadband(float *value, float tres, float max) {
	if (fabsf(*value) < tres) {
		*value = 0.0;
	} else {
		float k = max / (max - tres);
		if (*value > 0.0) {
			*value = k * *value + max * (1.0 - k);
		} else {
			*value = -(k * -*value + max * (1.0 - k));
		}

	}
}

/**
 * A system locking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysLock and chSysLockFromIsr are the same
 * for this port.
 */
void utils_sys_lock_cnt(void) {
	if (!sys_lock_cnt) {
		chSysLock();
	}
	sys_lock_cnt++;
}

/**
 * A system unlocking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysUnlock and chSysUnlockFromIsr are the same
 * for this port.
 */
void utils_sys_unlock_cnt(void) {
	if (sys_lock_cnt) {
		sys_lock_cnt--;
		if (!sys_lock_cnt) {
			chSysUnlock();
		}
	}
}
