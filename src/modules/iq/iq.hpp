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
#include <time.h>
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
#define MODE_TEST 2
#define MODE_CALIBRATION 3
#define TIMEOUT 0.003
#define SLEEP_TIME 30000
#define TOPICS_TIME 150
// #define TEST_SPEED_MIN 0.2
// #define TEST_SPEED_MAX 0.7
// #define TEST_SPEED_N 2
// #define TEST_PULSE_N 2
// #define TEST_PULSE_MIN 0.0
// #define TEST_PULSE_MAX 1.0
// #define TEST_TIME 2

struct parameter{
    char* name;
    float value;
};

struct orb_topic {
    int fd;
    const orb_metadata id;
    unsigned int interval;
};

extern "C" __EXPORT int iq_main(int argc, char *argv[]);
static volatile bool thread_should_exit = false;   /**< Daemon exit flag */
static volatile bool thread_running = false;   /**< Daemon status flag */
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
int iq_thread_main(int argc, char *argv[]);
void enter_coast_mode();
void send_commands(float *actuator_control);
int init_system(float timeout, int sleep_time);
int set_parameter(char *param_name, float *param_value);
// int set_parameter2(parameter *param_struct);

bool is_armed = false;
bool was_armed = false;
bool test_started = false;
short int mode = MODE_FLIGHT;

// param values to be loaded
float max_speed_value = 0.01f;
float min_pulse_volts_value = 0.01;
float max_pulse_volts_value = 0.01;
float max_yaw_value = 0.01;
float min_roll_vel_value = 0.01;
float max_roll_vel_value = 0.01;
float max_roll_inclination_value = 0.01;

// other variables
int error_counter = 0;
float velocity = 0, x_roll = 0, y_pitch = 0, z_yaw = 0;//, roll_speed = 0;
float amplitude = 0, phase = 0;
float q[4];
float gyro[3];
float acc[3];
float control[5];// = {0, 0, 0, 0, 0};

// communication variables
GenericInterface com;
char *uart_name1;
PropellerMotorControlClient pmc1(0);
PropellerMotorControlClient pmc2(1);
VoltageSuperPositionClient  vsc1(0);
VoltageSuperPositionClient  vsc2(1);
SystemControlClient sys(0);
int serial_fds = -1;

int actuator_arm_sub_fd;
int actuator_ctrl_manual_sub_fd;
// int actuator_ctrl_sub_fd;
// int sensor_combined_sub_fd;
// int vehicle_attitude_sub_fd;

px4_pollfd_struct_t fds[2];

// orb_topic actuator_arm{}
