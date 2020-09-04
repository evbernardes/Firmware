/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file iq.cpp
 * Minimal application example for an underactuated propeller vehicle powered by IQinetics motors
 *
 * @author Matthew Piccoli <matt@iqinetics.com>
 */

#include "iq.hpp"

int iq_thread_main(int argc, char *argv[])
{
	PX4_INFO("IQinetics Underactuated Propeller Thread Loading");

    // Check input arguments
    char *uart_name1;
        if (argc < 2) {
        uart_name1 = (char *) ("/dev/ttyS3");
        PX4_WARN("No serial port argument given, using default: /dev/ttyS3");
        // return 1;
    } else if (argc > 2) {
        PX4_ERR("Argument error, correct start usage: iq start [port]");
        return 1;
    } else {
        uart_name1 = argv[1];
    }

    // int serial_fds = -1;

    if(setup_uart(uart_name1, serial_fds) == 0)
        PX4_INFO("Opened %s with fd %d", uart_name1, serial_fds);
    else
        return 1;

    set_parameters();

    // GenericInterface com;
    // PropellerMotorControlClient pmc1(0);
    // PropellerMotorControlClient pmc2(1);
    // VoltageSuperPositionClient  vsc1(0);
    // VoltageSuperPositionClient  vsc2(1);
    // SystemControlClient sys(0);
    // PX4_INFO("Clients created");

    pmc1.timeout_.set(com,0.002);
    pmc2.timeout_.set(com,0.002);
    sys.reboot_program_.set(com);
    send_msgs_to_uart(com, serial_fds);
    usleep(30000);
    PX4_INFO("Motors modules rebooted");

    // subscribe to actuator_controls_0 topic
    int actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    int actuator_arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3)); // manual control
    int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(actuator_ctrl_sub_fd, 1);
    orb_set_interval(actuator_arm_sub_fd, 1);
    orb_set_interval(input_rc_sub_fd, 1);
    orb_set_interval(actuator_ctrl_manual_sub_fd, 1);
    orb_set_interval(sensor_combined_sub_fd, 1);
    px4_pollfd_struct_t fds[] = {
        { .fd = actuator_arm_sub_fd,   .events = POLLIN },
        { .fd = actuator_ctrl_sub_fd,   .events = POLLIN },
        { .fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN },
        { .fd = sensor_combined_sub_fd,   .events = POLLIN },
        // { .fd = input_rc_sub_fd,   .events = POLLIN },
    };

	// initialize variables
    error_counter = 0;
    velocity = 0;
    x_roll = 0; y_pitch = 0; z_yaw = 0;
    amplitude = 0; phase = 0;
    thread_running = true;

	// main while loop for this thread
	while(!thread_should_exit)
	{
		int poll_ret = px4_poll(fds, 4, 25);

		// /* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a 10ms");
		}
		else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0)
			{
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;
		}
		else
		{
		  if (fds[0].revents & POLLIN)
		  {
		    // Get the actuator armed data
        struct actuator_armed_s actuator_arm_raw;
        orb_copy(ORB_ID(actuator_armed), actuator_arm_sub_fd, &actuator_arm_raw);

        is_armed = actuator_arm_raw.armed;
      }

      if (is_armed) {
        if ((mode == MODE_FLIGHT) && (fds[1].revents & POLLIN)) {
          // Get the FLIGHT MODE actuator control data
          struct actuator_controls_s actuator_raw;
          orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
          // --------------------------------------------------------------------
          // Vehicle behavior goes here
          velocity  = actuator_raw.control[3]*max_speed_value;
          x_roll    = actuator_raw.control[0]*max_pulse_volts_value;
          y_pitch   = actuator_raw.control[1]*max_pulse_volts_value;
          z_yaw     = actuator_raw.control[2]*max_yaw_value;
          z_yaw     = 0;
          amplitude = sqrt(x_roll * x_roll + y_pitch * y_pitch);
          phase = atan2(x_roll,y_pitch);

        } else if ((mode == MODE_ROLL) && (fds[2].revents & POLLIN)) {
          phase = 0;
          amplitude = 0;
          velocity = 0; // uses z_yaw to send rolling command (because of the inverted sign)

          // Get the MANUAL MODE actuator control data
          struct actuator_controls_s actuator_raw;
          orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
          z_yaw  = actuator_raw.control[1]*max_roll_vel_value;

          if (fabs(z_yaw) < (double)min_roll_vel_value)
            z_yaw = 0;

        }

        send_commands();
      }

      if(was_armed && !is_armed) enter_coast_mode();

      was_armed = is_armed;

		}
	}
	PX4_INFO("exiting");
  thread_running = false;

  fflush(stdout);
	return 0;
}

/**
 * Find all parameters, load their values and set them to the global variables
 */
void set_parameters() {
  // load params
  param_t max_speed_param;
  param_t max_pulse_volts_param;
  param_t max_yaw_param;
  param_t min_roll_vel_param;
  param_t max_roll_vel_param;
  param_t max_roll_inclination_param;

  max_speed_param = param_find("PROP_MAX_SPEED");
  max_pulse_volts_param = param_find("PROP_MAX_PULSE");
  max_yaw_param = param_find("PROP_MAX_YAW");
  min_roll_vel_param = param_find("MIN_ROLL_VEL");
  max_roll_vel_param = param_find("MAX_ROLL_VEL");
  max_roll_inclination_param = param_find("MAX_ROLL_INC");

  if (max_speed_param != PARAM_INVALID)
    param_get(max_speed_param, &max_speed_value);
  else
    PX4_WARN("PROP_MAX_SPEED param invalid");

  if (max_pulse_volts_param != PARAM_INVALID)
    param_get(max_pulse_volts_param, &max_pulse_volts_value);
  else
    PX4_WARN("PROP_MAX_PULSE param invalid");

  if (max_yaw_param != PARAM_INVALID)
    param_get(max_yaw_param, &max_yaw_value);
  else
    PX4_WARN("PROP_MAX_YAW param invalid");

  if (min_roll_vel_param != PARAM_INVALID)
    param_get(min_roll_vel_param, &min_roll_vel_value);
  else
    PX4_WARN("MIN_ROLL_VEL param invalid");

  if (max_roll_vel_param != PARAM_INVALID)
    param_get(max_roll_vel_param, &max_roll_vel_value);
  else
    PX4_WARN("MAX_ROLL_VEL param invalid");

  if (max_roll_inclination_param != PARAM_INVALID)
    param_get(max_roll_inclination_param, &max_roll_inclination_value);
  else
    PX4_WARN("MAX_ROLL_INC param invalid");

  PX4_INFO("PARAMS Loaded");
}

/* Startup Functions */

/**
 * Print correct usage.
 */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr, "usage: iq {start|stop|status|switch}\n\n");
}

/**
 * Send commands to the motors
 */
void send_commands() {
    pmc1.ctrl_velocity_.set(com,velocity - z_yaw); // INDEX_THROTTLE = 3, INDEX_YAW = 2
    vsc1.phase_.set(com,phase);
    vsc1.amplitude_.set(com,amplitude);

    pmc2.ctrl_velocity_.set(com,velocity + z_yaw); // INDEX_THROTTLE = 3, INDEX_YAW = 2
    vsc2.phase_.set(com,phase);
    vsc2.amplitude_.set(com,amplitude);

    int send_ret = send_msgs_to_uart(com, serial_fds);
    if(send_ret != 0) PX4_WARN("serial1 send error %d", send_ret);
}

/**
 * put motors in coast mode
 */
void enter_coast_mode() {
    pmc1.ctrl_coast_.set(com);
    pmc2.ctrl_coast_.set(com);
    PX4_INFO("System disarmed, motors put in coast mode");

    int send_ret = send_msgs_to_uart(com, serial_fds);
    if(send_ret != 0)
        PX4_WARN("serial1 send error %d", send_ret);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int iq_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage("missing command");
    return 1;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      PX4_INFO("already running\n");
      /* this is not an error */
      return 0;
    }

    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawning");

    thread_should_exit = false;
    daemon_task = px4_task_spawn_cmd("iq",
             SCHED_DEFAULT,
             SCHED_PRIORITY_DEFAULT - 1,
             5000,
             iq_thread_main,
             (argv) ? (char *const *)&argv[2] : (char *const *)NULL);

    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawned");
    return 0;
  }

  if (!strcmp(argv[1], "switch")) {
    if (argc < 3) {
      PX4_INFO("usage: iq switch {flight|roll|calibration}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (!strcmp(argv[2], "flight")) {
      if (mode == MODE_FLIGHT) {
        PX4_INFO("already in flight mode\n");
        return 0;
      } else {
        PX4_INFO("entering flight mode\n");
        mode = MODE_FLIGHT;
        return 0;
      }

    } else if (!strcmp(argv[2], "roll")) {
      if (mode == MODE_ROLL) {
        PX4_INFO("already in roll mode\n");
        return 0;
      } else {
        PX4_INFO("entering roll mode\n");
        mode = MODE_ROLL;
        return 0;
      }
    }

    else {
      PX4_INFO("usage: iq switch [flight:roll]\n");
      return -1;
    }
  }

  if (!strcmp(argv[1], "stop")) {
    if(is_armed)
    {
      PX4_ERR("cannot stop while armed");
      return -1;
    }

    PX4_INFO("stopping");
    thread_should_exit = true;
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      if(mode == MODE_FLIGHT)
        PX4_INFO("running in flight mode");
      else if(mode == MODE_ROLL)
        PX4_INFO("running roll mode");

    } else {
      PX4_INFO("stopped");
    }

    return 0;
  }

  PX4_ERR("unrecognized command");
  return 1;
}
