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
 * @file iq_calibrate.cpp
 * Minimal application example for an underactuated propeller vehicle powered by IQinetics motors
 *
 * @author Matthew Piccoli <matt@iqinetics.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <parameters/param.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
// #include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#define clrscr() PX4_INFO("\e[1;1H\e[2J")

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>

#include "generic_interface.hpp"
#include "voltage_super_position_client.hpp"
#include "system_control_client.hpp"
#include "propeller_motor_control_client.hpp"

#define MODE_FLIGHT 0
#define MODE_ROLL 1

extern "C" __EXPORT int iq_calibrate_main(int argc, char *argv[]);
static volatile bool thread_should_exit = false;   /**< Daemon exit flag */
static volatile bool thread_running = false;   /**< Daemon status flag */
static volatile int daemon_task;       /**< Handle of daemon task / thread */

// int iq_thread_main(int argc, char *argv[]);
int send_msgs_to_uart(GenericInterface& com, int serial_fd);
int setup_uart(char *uart_name, int &serial_fd);
void set_parameters();
double get_norm(double *array, int length);
void normalize(double *array, int length);

// declade global serial_fds and serial interface
// this way they can be used outside of the main thread
// int serial_fds;
// GenericInterface com;

// PropellerMotorControlClient pmc1(0);
// PropellerMotorControlClient pmc2(1);
// VoltageSuperPositionClient  vsc1(0);
// VoltageSuperPositionClient  vsc2(1);
// SystemControlClient sys(0);
// PX4_INFO("Clients created");
bool is_armed = false;
bool was_armed = false;
short int mode = MODE_FLIGHT;

float max_speed_value = 0.0f;
float max_pulse_volts_value = 0.0f;
float max_yaw_value = 0.0f;
float min_roll_vel_value = 0.0f;
float max_roll_vel_value = 0.0f;
float max_roll_inclination_value = 0.0f;

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */

int iq_calibrate_main(int argc, char *argv[])
{
	PX4_INFO("IQinetics Underactuated Propeller Calibration Thread Loading");

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

    int serial_fds = -1;

    if(setup_uart(uart_name1, serial_fds) == 0)
        PX4_INFO("Opened %s with fd %d", uart_name1, serial_fds);
    else
        return 1;

    set_parameters();

    GenericInterface com;

    PropellerMotorControlClient pmc1(0);
    PropellerMotorControlClient pmc2(1);
    VoltageSuperPositionClient  vsc1(0);
    VoltageSuperPositionClient  vsc2(1);
    SystemControlClient sys(0);
    PX4_INFO("Clients created");

    pmc1.timeout_.set(com,0.002);
    pmc2.timeout_.set(com,0.002);
    sys.reboot_program_.set(com);
    send_msgs_to_uart(com, serial_fds);
    usleep(30000);
    PX4_INFO("Motors modules rebooted");

	// subscribe to actuator_controls_0 topic
	int actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	// subscribe to actuator_armed topic
	int actuator_arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
	int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3)); // manual control
    int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

    /* limit the update rate to 1 kHz */
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
    };

	// initialize variables
    bool initial_attitude = false;
    int n = 0;
	int error_counter = 0;
    float velocity = 0, x_roll = 0, y_pitch = 0, z_yaw = 0;//, roll_speed = 0;
    float amplitude = 0, phase = 0;
    thread_running = true;
    double gyro[3], acc[3], acc0[3], q[4];

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
                // was_armed = true;
                // Get sensor data
                struct sensor_combined_s sensor_combined_raw;
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined_raw);
                gyro[0] = sensor_combined_raw.gyro_rad[0];
                gyro[1] = sensor_combined_raw.gyro_rad[1];
                gyro[2] = sensor_combined_raw.gyro_rad[2];
                double omega = get_norm(gyro,3);
                acc[0] = sensor_combined_raw.accelerometer_m_s2[0];
                acc[1] = sensor_combined_raw.accelerometer_m_s2[1];
                acc[2] = sensor_combined_raw.accelerometer_m_s2[2];
                normalize(acc,3);
                // double norm = get_norm(acc,3);
                // acc[0] /= norm;
                // acc[1] /= norm;
                // acc[2] /= norm;

                if(!initial_attitude){
                    acc0[0] = acc[0];
                    acc0[1] = acc[1];
                    acc0[2] = acc[2];
                    initial_attitude = true;
                }

                q[0] = (1-acc[2])*(1-acc0[2]) + acc[0]*acc0[0] + acc[1]*acc0[1];
                q[1] = -acc0[1]*(1-acc[2]) + acc[1]*(1-acc0[2]);
                // q[2] = +acc0[0]*(1-acc[2]) - acc[0]*(1-acc0[2]);
                q[2] = 0;
                q[3] = acc0[0]*acc[1] - acc[0]*acc0[1];
                // double q_norm = get_norm(q,4);
                // q[0] /= q_norm;
                // q[1] /= q_norm;
                // q[2] /= q_norm;
                // q[3] /= q_norm;
                normalize(q,4);
                double direction = atan2(-q[1],q[2]);
                double inclination = 2*atan2(sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]),q[0]);

                // Continue
                if (fds[1].revents & POLLIN) {
                    // Get the FLIGHT MODE actuator control data
                    struct actuator_controls_s actuator_raw;
                    orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
                    // --------------------------------------------------------------------
                    // Vehicle behavior goes here
                    max_speed_value = 100;
                    max_pulse_volts_value = 0;
                    velocity  = actuator_raw.control[3]*max_speed_value;
                    x_roll    = actuator_raw.control[0]*max_pulse_volts_value;
                    y_pitch   = actuator_raw.control[1]*max_pulse_volts_value;
                    // z_yaw     = actuator_raw.control[2]*max_yaw_value;
                    z_yaw     = 0;
                    amplitude = sqrt(x_roll * x_roll + y_pitch * y_pitch);
                    phase = atan2(x_roll,y_pitch);
                    PX4_INFO("direction = %.2f | phase = %.2f | inclination = %.2f  | omega = %.2f |", direction * M_RAD_TO_DEG, phase * M_RAD_TO_DEG, inclination * M_RAD_TO_DEG, omega * M_RAD_TO_DEG);


                    // send commands to motors
                    pmc1.ctrl_velocity_.set(com,velocity - z_yaw); // INDEX_THROTTLE = 3, INDEX_YAW = 2
                    vsc1.phase_.set(com,phase);
                    vsc1.amplitude_.set(com,amplitude);

                    pmc2.ctrl_velocity_.set(com,velocity + z_yaw); // INDEX_THROTTLE = 3, INDEX_YAW = 2
                    vsc2.phase_.set(com,phase);
                    vsc2.amplitude_.set(com,amplitude);

                    int send_ret = send_msgs_to_uart(com, serial_fds);
                    if(send_ret != 0)
                    PX4_WARN("serial1 send error %d", send_ret);
                }

            } else if(was_armed) {
                // put motors in coast mode
                pmc1.ctrl_coast_.set(com);
                pmc2.ctrl_coast_.set(com);
                PX4_INFO("System disarmed, motors put in coast mode");
                int send_ret = send_msgs_to_uart(com, serial_fds);
                thread_should_exit = true;
                if(send_ret != 0)
                    PX4_WARN("serial1 send error %d", send_ret);
            } else {
                PX4_INFO("System disarmed, standing by...");
            }

            was_armed = is_armed;

		}
	}
	PX4_INFO("exiting");
    thread_running = false;

    fflush(stdout);
	return 0;
}

int send_msgs_to_uart(GenericInterface& comm, int serial_fd)
{
  // This buffer is for passing around messages.
  uint8_t communication_buffer[256];
  // Stores length of message to send or receive
  uint8_t communication_length;

  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(serial_fd >= 0 && comm.GetTxBytes(communication_buffer,communication_length))
  {
    //TODO::do the write in a while loop, decrementing com_length by written and incrementing buffer address
    uint8_t written = ::write(serial_fd, communication_buffer, communication_length);
    ::fsync(serial_fd);
    if(written != communication_length)
      return -1;
    return 0;
  }
  return -2;
}

/**
 *
 */
void set_parameters()
{
  // load params
  param_t max_speed_param;
  param_t max_pulse_volts_param;
  param_t max_yaw_param;
  param_t min_roll_vel_param;
  param_t max_roll_vel_param;
  param_t max_roll_inclination_param;

  // float max_speed_value = 0.0f;
  // float max_pulse_volts_value = 0.0f;
  // float max_yaw_value = 0.0f;
  // float min_roll_vel_value = 0.0f;
  // float max_roll_vel_value = 0.0f;
  // float max_roll_inclination_value = 0.0f;

  max_speed_param = param_find("PROP_MAX_SPEED");
  max_pulse_volts_param = param_find("PROP_MAX_PULSE");
  max_yaw_param = param_find("PROP_MAX_YAW");
  min_roll_vel_param = param_find("MIN_ROLL_VEL");
  max_roll_vel_param = param_find("MAX_ROLL_VEL");
  max_roll_inclination_param = param_find("MAX_ROLL_INC");

  if (max_speed_param != PARAM_INVALID) {
    param_get(max_speed_param, &max_speed_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_SPEED param invalid");
  }

  if (max_pulse_volts_param != PARAM_INVALID) {
    param_get(max_pulse_volts_param, &max_pulse_volts_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_PULSE param invalid");
  }

  if (max_yaw_param != PARAM_INVALID) {
    param_get(max_yaw_param, &max_yaw_value);
  }
  else
  {
    PX4_WARN("PROP_MAX_YAW param invalid");
  }

  if (min_roll_vel_param != PARAM_INVALID) {
    param_get(min_roll_vel_param, &min_roll_vel_value);
  }
  else
  {
    PX4_WARN("MIN_ROLL_VEL param invalid");
  }

  if (max_roll_vel_param != PARAM_INVALID) {
    param_get(max_roll_vel_param, &max_roll_vel_value);
  }
  else
  {
    PX4_WARN("MAX_ROLL_VEL param invalid");
  }

  if (max_roll_inclination_param != PARAM_INVALID) {
    param_get(max_roll_inclination_param, &max_roll_inclination_value);
  }
  else
  {
    PX4_WARN("MAX_ROLL_INC param invalid");
  }

  PX4_INFO("PARAMS Loaded");
}

/**
 * setup_uart initializes a uart port to 115200 8N1
 * uart_name is the port string descriptor (/dev/ttySx)
 * serial_fd is the resulting file descriptor for the port
 * returns 0 if successful, -1 if setup is unable to setup the port
 */
int setup_uart(char *uart_name, int &serial_fd)
{

  PX4_INFO("opening port %s", uart_name);

  serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

  if (serial_fd < 0) {
    PX4_WARN("failed to open port: %s", uart_name);
    return -1;
  }

  /* Try to set baud rate */
  struct termios uart_config;
  struct termios uart_config_original;
  int termios_state;

  /* Back up the original uart configuration to restore it after exit */
  if ((termios_state = tcgetattr(serial_fd, &uart_config_original)) < 0) {
    PX4_WARN("ERR GET CONF %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  tcgetattr(serial_fd, &uart_config);
  uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  uart_config.c_iflag &= ~IGNBRK;         // disable break processing
  uart_config.c_lflag = 0;                // no signaling chars, no echo,
  uart_config.c_iflag &= ~INLCR;
  uart_config.c_iflag &= ~ICRNL;
  uart_config.c_oflag &= ~OCRNL;
  uart_config.c_oflag &= ~ONLCR;
  // no canonical processing
  uart_config.c_oflag = 0;                // no remapping, no delays
  uart_config.c_cc[VMIN]  = 0;            // read doesn't block
  uart_config.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  uart_config.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  uart_config.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  uart_config.c_cflag |= 0;
  uart_config.c_cflag &= ~CSTOPB;
  uart_config.c_cflag &= ~CRTSCTS;

  /* Set baud rate */
  const speed_t speed = B115200;
  // const speed_t speed = B57600;

  /* Set baud rate */
  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
    PX4_WARN("ERR SET BAUD %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
    PX4_WARN("ERR SET CONF %s\n", uart_name);
    close(serial_fd);
    return -1;
  }
  return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: iq {start|stop|status|switch}\n\n");
}

double get_norm(double *array, int length)
{
    double result = 0;
    for(int i = 0; i < length; i++){
        result += array[i]*array[i];
    }
    return sqrt(result);
}

void normalize(double *array, int length)
{
    double norm = get_norm(array, length);
    for(int i = 0; i < length; i++){
        array[i] /= norm;
    }
}
