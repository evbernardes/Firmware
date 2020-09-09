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

__EXPORT int iq_test_main(int argc, char *argv[]);

iq_test_main(int argc, char *argv[]){

}
