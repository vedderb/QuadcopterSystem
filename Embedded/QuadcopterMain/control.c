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
#include "stm32f4xx_conf.h"

#include <math.h>
#include <string.h>

#include "utils.h"
#include "mpu9150.h"
#include "actuator.h"
#include "packet_handler.h"
#include "pwm_esc.h"
#include "control.h"
#include "MahonyAHRS.h"
#include "pos.h"
#include "packet_handler_int.h"
#include "navigation.h"

// Default control parameters
#define DEFAULT_ROLL_P_GAIN				0.8
#define DEFAULT_ROLL_I_GAIN				1.0
#define DEFAULT_ROLL_D_GAIN_PROCESS		0.3
#define DEFAULT_ROLL_D_GAIN_ERROR		0.2

#define DEFAULT_PITCH_P_GAIN			0.8
#define DEFAULT_PITCH_I_GAIN			1.0
#define DEFAULT_PITCH_D_GAIN_PROCESS	0.3
#define DEFAULT_PITCH_D_GAIN_ERROR		0.2

#define DEFAULT_YAW_P_GAIN				3.0
#define DEFAULT_YAW_I_GAIN				0.2
#define DEFAULT_YAW_D_GAIN_PROCESS		0.4
#define DEFAULT_YAW_D_GAIN_ERROR		0.2

// Gain for the received values from the controller
#define ROLL_ITERATION_GAIN				1.0
#define PITCH_ITERATION_GAIN			1.0
#define YAW_ITERATION_GAIN				0.6

// Other parameters
#define ITERATION_TIMER_FREQ			50000
#define INPUT_TIMEOUT_MS				1000
#define POWER_OVERRIDE_TIMEOUT_MS		500
#define AUTOPILOT_TIMEOUT_MS			1000
#define MIN_THROTTLE					0.1
#define THROTTLE_OVERRIDE_LIM			0.85

// Private variables
volatile uint8_t run_control_loop = 0;
volatile int use_advanced_mode = 0;

static CONTROL_PARAMETERS_t control_parameters;

static volatile float roll_now = 0.0;
static volatile float pitch_now = 0.0;
static volatile float yaw_now = 0.0;

static volatile float roll_goal = 0;
static volatile float pitch_goal = 0;
static volatile float yaw_goal = 0;

static volatile float roll_offset = 0;
static volatile float pitch_offset = 0;
static volatile float yaw_offset = 0;

static THD_WORKING_AREA(control_thread_wa, 2048);

static volatile float power_override[4] = {0.0, 0.0, 0.0, 0.0};
static volatile int power_override_time = -1;
static volatile float last_iteration_delay_ms = 0.0;
static volatile int throttle_blocked = 0;
static volatile int autopilot_now = 0;

static volatile ATTITUDE_INFO att_no_mag, att_mag;
static volatile float accel_used[3];
static volatile float gyro_used[3];
static volatile float mag_used[3];

static float last_roll_process = 0;
static float last_roll_error = 0;
static float roll_integrator = 0;
static float last_pitch_process = 0;
static float last_pitch_error = 0;
static float pitch_integrator = 0;
static float last_yaw_process = 0;
static float last_yaw_error = 0;
static float yaw_integrator = 0;

// Private function prototypes
static void update_rc_angles(float roll_dec, float pitch_dec, float yaw_dec, float dt);
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);
static void run_iteration(float dt);
static THD_FUNCTION(control_thread, arg);
static void mpu_ready_func(void);

void control_init(void) {
	control_parameters.roll_p_gain = DEFAULT_ROLL_P_GAIN;
	control_parameters.roll_i_gain = DEFAULT_ROLL_I_GAIN;
	control_parameters.roll_d_gain_process = DEFAULT_ROLL_D_GAIN_PROCESS;
	control_parameters.roll_d_gain_error = DEFAULT_ROLL_D_GAIN_ERROR;

	control_parameters.pitch_p_gain = DEFAULT_PITCH_P_GAIN;
	control_parameters.pitch_i_gain = DEFAULT_PITCH_I_GAIN;
	control_parameters.pitch_d_gain_process = DEFAULT_PITCH_D_GAIN_PROCESS;
	control_parameters.pitch_d_gain_error = DEFAULT_PITCH_D_GAIN_ERROR;

	control_parameters.yaw_p_gain = DEFAULT_YAW_P_GAIN;
	control_parameters.yaw_i_gain = DEFAULT_YAW_I_GAIN;
	control_parameters.yaw_d_gain_process = DEFAULT_YAW_D_GAIN_PROCESS;
	control_parameters.yaw_d_gain_error = DEFAULT_YAW_D_GAIN_ERROR;

	MahonyAHRSInitAttitudeInfo((ATTITUDE_INFO*)&att_no_mag);
	MahonyAHRSInitAttitudeInfo((ATTITUDE_INFO*)&att_mag);

	// Iteration timer (ITERATION_TIMER_FREQ Hz)
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	uint16_t PrescalerValue = (uint16_t)((168e6 / 2) / ITERATION_TIMER_FREQ) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM6, ENABLE);

	chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), NORMALPRIO, control_thread, NULL);
	run_control_loop = 1;

	mpu9150_set_read_callback(mpu_ready_func);
}

CONTROL_PARAMETERS_t *control_get_pid_parameters(void) {
	return &control_parameters;
}

void control_get_orientation_now(float *roll, float *pitch, float *yaw) {
	*roll = roll_now;
	*pitch = pitch_now;
	*yaw = yaw_now;
}

void control_reset_orientation_offsets(void) {
	roll_offset = utils_angle_difference(roll_now + roll_offset, 0);
	pitch_offset = utils_angle_difference(pitch_now + pitch_offset, 0);
	yaw_offset = utils_angle_difference(yaw_now + yaw_offset, 0);
}

void control_set_yaw(float angle) {
	yaw_offset = utils_angle_difference(yaw_now + yaw_offset, angle);
	last_yaw_process = 0;
	last_yaw_error = 0;
	yaw_goal = angle;
}

void control_override_power(int motor, float power) {
	if (motor >= 0 && motor < 4) {
		power_override[motor] = power;
		power_override_time = POWER_OVERRIDE_TIMEOUT_MS;
		throttle_blocked = 0;
	}
}

float control_get_last_iteration_delay_ms(void) {
	return last_iteration_delay_ms;
}

static THD_FUNCTION(control_thread, arg) {
	(void) arg;
	chRegSetThreadName("control thread");

	for(;;) {
		float aux_dec = 0.0;
		int i;

		if (packet_handler_get_time_since_stick_update() < INPUT_TIMEOUT_MS) {
			aux_dec = packet_handler_get_channel_mapped(CH_POT_L);
		}

		if (aux_dec < 0.5) {
			use_advanced_mode = 0;
		} else {
			use_advanced_mode = 1;
		}

		if (power_override_time > 0) {
			power_override_time -= 10;

			for (i = 0;i < 4;i++) {
				pwm_esc_set(i, (uint8_t)(power_override[i] * 255.0));
			}
		}

		if (mpu9150_get_time_since_update() > 1000) {
			pwm_esc_set(255, 0);
		}

		chThdSleepMilliseconds(10);
	}
}

static void mpu_ready_func(void) {
	if (run_control_loop) {
		float accel[3] = {0, 0, 0};
		float gyro[3] = {0, 0, 0};
		float mag[3] = {0, 0, 0};
		float dt = 0.0;

		mpu9150_get_accel_gyro_mag(accel, gyro, mag);
		dt = (float)TIM6->CNT / (float)ITERATION_TIMER_FREQ;
		TIM6->CNT = 0;

		update_orientation_angles(accel, gyro, mag, dt);
		last_iteration_delay_ms = dt * 1000.0;

		run_iteration(dt);
	}
}

static void run_iteration(float dt) {
	float roll_dec = 0.0;
	float pitch_dec = 0.0;
	float yaw_dec = 0.0;
	float throttle_dec = 0.0;
	int lost_signal = 1;

	if (packet_handler_get_time_since_stick_update() < INPUT_TIMEOUT_MS) {
		throttle_dec = packet_handler_get_channel_mapped(CH_THROTTLE);
		roll_dec = packet_handler_get_channel_mapped(CH_ROLL);
		pitch_dec = packet_handler_get_channel_mapped(CH_PITCH);
		yaw_dec = packet_handler_get_channel_mapped(CH_YAW);
		lost_signal = 0;
	} else {
		packet_handler_zero_channels();
	}

	if (throttle_dec < 0.15 && !lost_signal && throttle_blocked) {
		throttle_blocked = 0;
	}

	static systime_t last_signal_time = 0;
	if (!lost_signal) {
		last_signal_time = chVTGetSystemTime();
	}

	if (power_override_time > 0) {
		return;
	}

	POS_STATE_t *pos_state = pos_get_state();

	static int was_override = 0;
	static float prev_throttle = 0;

	if (throttle_dec > THROTTLE_OVERRIDE_LIM &&
			packet_handler_int_get_time_since_update_altitude() < AUTOPILOT_TIMEOUT_MS) {
		static float override_x = 0.0;
		static float override_y = 0.0;
		autopilot_now = 1;

		if (!was_override) {
			override_x = pos_state->px;
			override_y = pos_state->py;

			throttle_dec = 0.0;
			was_override = 1;
			navigation_override_xy(override_x, override_y);
//			navigation_override_z(pos_state->pz);
			navigation_override_z(0.8);
		} else if (quad_config.quad_id == 0) {
			float roll_db = roll_dec;
			float pitch_db = pitch_dec;

			utils_deadband(&roll_db, 0.3, 1.0);
			utils_deadband(&pitch_db, 0.3, 1.0);

			override_x += roll_db * dt * 1.0;
			override_y -= pitch_db * dt * 1.0;

			utils_truncate_number(&override_x, quad_config.map_lim.min_x + POS_REG_MARGIN,
					quad_config.map_lim.max_x - POS_REG_MARGIN);
			utils_truncate_number(&override_y, quad_config.map_lim.min_y + POS_REG_MARGIN,
					quad_config.map_lim.max_y - POS_REG_MARGIN);

			navigation_override_xy(override_x, override_y);
		}
	} else {
		if (throttle_dec > THROTTLE_OVERRIDE_LIM && (was_override || prev_throttle < MIN_THROTTLE)) {
			throttle_blocked = 1;
		}

		was_override = 0;
		autopilot_now = 0;

		if (lost_signal && prev_throttle >= MIN_THROTTLE) {
			if (pos_state->pz > 0.10 &&
					(chVTTimeElapsedSinceX(last_signal_time) / CH_CFG_ST_FREQUENCY) < 5) {
				navigation_override_z(0.08);
			} else {
				navigation_override_xy_stop();
				navigation_override_z_stop();
			}
		} else {
			navigation_override_xy_stop();
			navigation_override_z_stop();
		}
	}

	navigation_run_xy(&roll_dec, &pitch_dec, dt);
	navigation_run_z(&throttle_dec);

	prev_throttle = throttle_dec;

	if (throttle_blocked) {
		throttle_dec = 0.0;
	}

	// Set roll, pitch and yaw goals to the current value if the throttle
	// just got above the threshold.
	static int was_below_tres = 0;
	if (throttle_dec < MIN_THROTTLE) {
		was_below_tres = 1;
		pwm_esc_set(255, 0);
		return;
	} else {
		if (was_below_tres) {
			roll_goal = roll_now;
			pitch_goal = pitch_now;
			yaw_goal = yaw_now;
			roll_integrator = 0.0;
			pitch_integrator = 0.0;
			yaw_integrator = 0.0;
		}

		was_below_tres = 0;
	}

//	yaw_goal = 0.0; // TODO!

	// Update goal coordinate system based on stick input
	update_rc_angles(roll_dec, pitch_dec, yaw_dec, dt);

	float roll_error = utils_angle_difference(roll_goal, roll_now);
	float pitch_error = utils_angle_difference(pitch_goal, pitch_now);
	float yaw_error = utils_angle_difference(yaw_goal, yaw_now);

	roll_error /= 360.0;
	pitch_error /= 360.0;
	yaw_error /= 360.0;

	// Run integration
	roll_integrator += roll_error * dt;
	pitch_integrator += pitch_error * dt;
	yaw_integrator += yaw_error * dt;
	// Prevent wind-up
	utils_truncate_number(&roll_integrator, -1.0 / control_parameters.roll_i_gain, 1.0 / control_parameters.roll_i_gain);
	utils_truncate_number(&pitch_integrator, -1.0 / control_parameters.pitch_i_gain, 1.0 / control_parameters.pitch_i_gain);
	utils_truncate_number(&yaw_integrator, -1.0 / control_parameters.yaw_i_gain, 1.0 / control_parameters.yaw_i_gain);

	float d_roll_sample_process = -utils_angle_difference(roll_now, last_roll_process) / dt;
	float d_roll_sample_error = (roll_error - last_roll_error) / dt;
	float d_pitch_sample_process = -utils_angle_difference(pitch_now, last_pitch_process) / dt;
	float d_pitch_sample_error = (pitch_error - last_pitch_error) / dt;
	float d_yaw_sample_process = -utils_angle_difference(yaw_now, last_yaw_process) / dt;
	float d_yaw_sample_error = (yaw_error - last_yaw_error) / dt;

	last_roll_process = roll_now;
	last_roll_error = roll_error;
	last_pitch_process = pitch_now;
	last_pitch_error = pitch_error;
	last_yaw_process = yaw_now;
	last_yaw_error = yaw_error;

	d_roll_sample_process /= 360.0;
	d_pitch_sample_process /= 360.0;
	d_yaw_sample_process /= 360.0;

	float roll_out = roll_error * control_parameters.roll_p_gain +
			roll_integrator * control_parameters.roll_i_gain +
			d_roll_sample_process * control_parameters.roll_d_gain_process +
			d_roll_sample_error * control_parameters.roll_d_gain_error;
	float pitch_out = pitch_error * control_parameters.pitch_p_gain +
			pitch_integrator * control_parameters.pitch_i_gain +
			d_pitch_sample_process * control_parameters.pitch_d_gain_process +
			d_pitch_sample_error * control_parameters.pitch_d_gain_error;
	float yaw_out = yaw_error * control_parameters.yaw_p_gain +
			yaw_integrator * control_parameters.yaw_i_gain +
			d_yaw_sample_process * control_parameters.yaw_d_gain_process +
			d_yaw_sample_error * control_parameters.yaw_d_gain_error;

	utils_truncate_number(&roll_out, -1.0, 1.0);
	utils_truncate_number(&pitch_out, -1.0, 1.0);
	utils_truncate_number(&yaw_out, -1.0, 1.0);

	// Compensate throttle for roll and pitch
	const float tan_roll = tanf(roll_now * M_PI / 180.0);
	const float tan_pitch = tanf(pitch_now * M_PI / 180.0);
	const float tilt_comp_factor = sqrtf(tan_roll * tan_roll + tan_pitch * tan_pitch + 1);

	throttle_dec *= tilt_comp_factor;
	utils_truncate_number(&throttle_dec, 0.0, 1.0);

	actuator_set_output(throttle_dec, roll_out, pitch_out, yaw_out);
}

static void update_rc_angles(float roll_dec, float pitch_dec, float yaw_dec, float dt) {
	if (fabsf(yaw_dec) < 0.05) {
		yaw_dec = 0.0;
	}

	float roll = roll_goal + roll_dec * ROLL_ITERATION_GAIN * dt * 250.0;
	float pitch = pitch_goal + pitch_dec * PITCH_ITERATION_GAIN * dt * 250.0;
	float yaw = yaw_goal + yaw_dec * YAW_ITERATION_GAIN * dt * 250.0;

	if (use_advanced_mode) {
		// Integrate towards the current angle slowly
		roll = utils_weight_angle(roll, roll_now, 0.995);
		pitch = utils_weight_angle(pitch, pitch_now, 0.995);
	} else {
		// A trick!
		roll = utils_weight_angle(roll, -roll_now, 0.995);
		pitch = utils_weight_angle(pitch, -pitch_now, 0.995);
	}

	utils_norm_angle(&roll);
	utils_norm_angle(&pitch);
	utils_norm_angle(&yaw);

	roll_goal = roll;
	pitch_goal = pitch;
	yaw_goal = yaw;
}

/*
 * Update the orientation based on IMU values. Notice that the magnetometer only
 * affects the YAW axis. This can surely be done more efficiently by not having two
 * complete sets of quaternions that are updated separately, but it does not really
 * matter.
 */
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt) {
	gyro[0] = gyro[0] * M_PI / 180.0;
	gyro[1] = gyro[1] * M_PI / 180.0;
	gyro[2] = gyro[2] * M_PI / 180.0;

	// Swap X and Y to match the accelerometer of the MPU9150
	float mag_tmp[3];
	mag_tmp[0] = mag[1];
	mag_tmp[1] = -mag[0];
	mag_tmp[2] = mag[2];

	accel_used[0] = accel[0];
	accel_used[1] = accel[1];
	accel_used[2] = accel[2];
	gyro_used[0] = gyro[0];
	gyro_used[1] = gyro[1];
	gyro_used[2] = gyro[2];
	mag_used[0] = mag_tmp[0];
	mag_used[1] = mag_tmp[1];
	mag_used[2] = mag_tmp[2];

	if (mpu9150_mag_updated() && fabsf(roll_now) < 25.0 && fabsf(pitch_now) < 25.0) {
		MahonyAHRSupdate(gyro, accel, mag_tmp, dt, (ATTITUDE_INFO*)&att_mag);
	} else {
		MahonyAHRSupdateIMU(gyro, accel, dt, (ATTITUDE_INFO*)&att_mag);
	}

	MahonyAHRSupdateIMU(gyro, accel, dt, (ATTITUDE_INFO*)&att_no_mag);

	roll_now = MahonyAHRSGetRoll((ATTITUDE_INFO*)&att_no_mag) * 180 / M_PI - roll_offset;
	pitch_now = MahonyAHRSGetPitch((ATTITUDE_INFO*)&att_no_mag) * 180 / M_PI - pitch_offset;
	yaw_now = MahonyAHRSGetYaw((ATTITUDE_INFO*)&att_mag) * 180 / M_PI - yaw_offset;

	pos_update(roll_now, pitch_now, yaw_now, dt);
}

/**
 * Get the accelerometer, gyro and magnetometer values used in the last iteration.
 */
void control_get_accel_gyro_mag_used(float *accel, float *gyro, float *mag) {
	accel[0] = accel_used[0];
	accel[1] = accel_used[1];
	accel[2] = accel_used[2];

	gyro[0] = gyro_used[0];
	gyro[1] = gyro_used[1];
	gyro[2] = gyro_used[2];

	mag[0] = mag_used[0];
	mag[1] = mag_used[1];
	mag[2] = mag_used[2];
}

/**
 *
 */
void control_update_initial_att(void) {
	MahonyAHRSupdateInitialOrientation((float*)accel_used, (float*)mag_used, (ATTITUDE_INFO*)&att_mag);
}

int control_has_autopilot(void) {
	return autopilot_now;
}
