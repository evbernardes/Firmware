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
#include "iq_helpers.cpp"

#define MODE_FLIGHT 0
#define MODE_ROLL 1
#define MODE_CALIBRATION 2

extern "C" __EXPORT int iq_main(int argc, char *argv[]);
static volatile bool thread_should_exit = false;   /**< Daemon exit flag */
static volatile bool thread_running = false;   /**< Daemon status flag */
static volatile int daemon_task;       /**< Handle of daemon task / thread */

int iq_thread_main(int argc, char *argv[]);
void enter_coast_mode();
void send_commands();
void set_parameters();
void calibrate();

bool is_armed = false;
bool was_armed = false;
short int mode = MODE_FLIGHT;

// param values to be loaded
float max_speed_value = 0.0f;
float max_pulse_volts_value = 0.0f;
float max_yaw_value = 0.0f;
float min_roll_vel_value = 0.0f;
float max_roll_vel_value = 0.0f;
float max_roll_inclination_value = 0.0f;

// other variables
int error_counter = 0;
float velocity = 0, x_roll = 0, y_pitch = 0, z_yaw = 0;//, roll_speed = 0;
float amplitude = 0, phase = 0;

// communication variables
GenericInterface com;
PropellerMotorControlClient pmc1(0);
PropellerMotorControlClient pmc2(1);
VoltageSuperPositionClient  vsc1(0);
VoltageSuperPositionClient  vsc2(1);
SystemControlClient sys(0);
int serial_fds = -1;

