#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
// #include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
// #include <uORB/topics/wind_estimate.h>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_global_position.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
// #include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

extern "C" __EXPORT int iq_test_main(int argc, char *argv[]);
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
int publish_actuators();
struct actuator_controls_s _actuators;
orb_advert_t _actuator_pub;
bool thread_running = false;
bool thread_should_exit = false;

float speed_min;
float speed_max;
int speed_n;
float pulse_min ;
float pulse_max;
int pulse_n;
// float speed_min = 0.2;
// float speed_max = 0.6;
// int speed_n = 3;
// float pulse_min = 0;
// float pulse_max = 0.6;
// int pulse_n = 5;

float test_phase;
float step_time;

int iq_test_thread_main(int argc, char *argv[])
{
    thread_running = true;
    float speed = 0.0;
    float pulse = 0.0;

    // float speed_min = 0.2;
    // float speed_max = 0.6;
    // int speed_n = 3;
    // float pulse_min = 0;
    // float pulse_max = 0.6;
    // int pulse_n = 5;
    // speed_min = 0.2;
    // speed_max = 0.6;
    // speed_n = 3;
    // pulse_min = 0;
    // pulse_max = 0.6;
    // pulse_n = 5;
    // float test_phase;
    // float step_time;

    bool thread_should_exit = false;
    int error_counter = 0;
    bool is_armed = true;
    bool was_armed = true;
    _actuators.control[0] = 0.0f;
    _actuators.control[1] = 0.0f;
    _actuators.control[2] = 0.0f;
    _actuators.control[3] = 0.0f;
    _actuators.control[4] = 1.0f;
    int start, stop;
    int i = 0, j = 0;
    start = 0;
    stop = step_time;

    PX4_INFO("Starting test...");

    while(!thread_should_exit){
        // speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;
        // pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
        // _actuators.control[3] = speed;
        // _actuators.control[0] = pulse*cos(test_phase);
        // _actuators.control[1] = pulse*sin(test_phase);

        if((stop - start)/1000000.0 > step_time){
            if(j == pulse_n - 1 & i == speed_n - 1){
                thread_should_exit = true;
            } else if (j == pulse_n - 1) {
                j = 0;
                i++;
            } else {
                j++;
            }
            speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;
            pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
            _actuators.control[3] = speed;
            _actuators.control[0] = pulse*cos(test_phase*M_DEG_TO_RAD);
            _actuators.control[1] = pulse*sin(test_phase*M_DEG_TO_RAD);
            start = hrt_absolute_time();
            PX4_INFO("i = %d/%d, j = %d/%d, time = %.2f",i+1,speed_n,j+1,pulse_n,start/1000000.0);
        }
        publish_actuators();
        stop = hrt_absolute_time();
    }

    _actuators.control[3] = 0.0;
    _actuators.control[0] = 0.0;
    _actuators.control[1] = 0.0;
    publish_actuators();
    PX4_INFO("Test finished, exiting.");
    thread_running = false;
    // fflush(stdout);
	// return 0;
}

int publish_actuators()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_controls_3), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_3), &_actuators);

		if (_actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

int iq_test_main(int argc, char *argv[]){

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

        // default values
        speed_min = 0.2;
        speed_max = 0.6;
        speed_n = 3;
        pulse_min = 0;
        pulse_max = 0.6;
        pulse_n = 5;
        test_phase = 0;
        step_time = 1.0;

        switch (argc)
        {
        case 10: step_time = atof(argv[9]);
        case 9: test_phase = atof(argv[8]);
        case 8: pulse_n = atoi(argv[7]);
        case 7: pulse_max = atof(argv[6]);
        case 6: pulse_min = atof(argv[5]);
        case 5: speed_n = atoi(argv[4]);
        case 4: speed_max = atof(argv[3]);
        case 3: speed_min = atof(argv[2]);
        case 2: break;
        // case 1: break;
        default:
            PX4_ERR("usage: iq_test start speed_min speed_min speed_n pulse_min pulse_max pulse_n test_phase step_time");
            return -1;
        }

        thread_should_exit = false;

        daemon_task = px4_task_spawn_cmd("iq_test",
                SCHED_DEFAULT,
                SCHED_PRIORITY_DEFAULT - 2,
                2000,
                iq_test_thread_main,
                //(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                //  (argv) ? (char *const *)&uart_name1 : (char *const *)NULL);
                (char *const *)NULL);

        PX4_INFO("Speed = [%.2f,%.2f], %d steps",speed_min,speed_max,speed_n);
        PX4_INFO("Pulse = [%.2f,%.2f], %d steps",pulse_min,pulse_max,pulse_n);
        PX4_INFO("Pulse test_phase = %.2f",test_phase);
        PX4_INFO("Step time = %.2fs",step_time);
        PX4_INFO("Test thread started");
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!thread_running) {
            PX4_INFO("already stopped\n");
            /* this is not an error */
        } else {
            PX4_INFO("stopping");
            thread_should_exit = true;
        }
        return 0;
    }

    PX4_ERR("unrecognized command");
    return 1;
}

/**
 * Print correct usage.
 */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr, "usage: iq_test {start|stop|status}\n\n");
}
