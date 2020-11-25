/*
 * iqinetics_params.c
 *
 *  Created on: Aug 14, 2017
 *      Author: Matthew Piccoli
 */

/**
 * Propeller min pulse
 *
 * The minimum pulse voltage amplitude (below that, amplitude goes to zero)
 *
 * @unit volts
 * @min 0
 * @max 11.1
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MIN_PULSE, 0.1f);

/**
 * Propeller max pulse
 *
 * The maximum pulse voltage amplitude
 *
 * @unit volts
 * @min 0
 * @max 11.1
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_PULSE, 3.0f);

/**
 * Propeller max speed
 *
 * The propeller's maximum average speed
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 100
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_SPEED, 650.0f);

/**
 * Propeller max yaw
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 10
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_YAW, 500.0f);


/**
 * Rolling mode minimum velocity
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(MIN_ROLL_VEL, 20.0f);

/**
 * Rolling mode maximum velocity
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(MAX_ROLL_VEL, 120.0f);

/**
 * Rolling mode roll angle safety
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(MAX_ROLL_INC, 0.78f); // pi / 4

/**
 * Phase of up motor
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(MOTOR_PHASE_UP, 0.0f); // 0
