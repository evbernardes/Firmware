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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
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
#include <time.h>

extern "C" __EXPORT int iq_test_main(int argc, char *argv[]);
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
int set_parameter(char *param_name, float *param_value);
int publish_actuators();
struct actuator_controls_s _actuators;
orb_advert_t _actuator_pub;
bool thread_running = false;
bool thread_should_exit = false;
bool thread_can_exit = false;

float stop_signal_time = 2.0;
float speed_min;
float speed_max;
int speed_n;
float pulse_min ;
float pulse_max;
int pulse_n;
float test_phase;
float step_time;
int plot_start;
int plot_step;
char filename[40] = "test";
// float q[4], acc[4], gyro[4];

int sensor_combined_sub_fd;
int vehicle_attitude_sub_fd;

int iq_test_thread_main(int argc, char *argv[])
{
    // set_parameter("PROP_MAX_SPEED", &max_speed_value);
    // set_parameter("PROP_MAX_PULSE", &max_pulse_volts_value);

    // set uORB topic subscribers
    sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    orb_set_interval(sensor_combined_sub_fd, 1);
    orb_set_interval(vehicle_attitude_sub_fd, 1);
    px4_pollfd_struct_t fds[] = {
        {.fd = sensor_combined_sub_fd,   .events = POLLIN },
        {.fd = vehicle_attitude_sub_fd,   .events = POLLIN }};

    thread_running = true;
    // thread_can_exit = false;
    thread_should_exit = false;
    int error_counter = 0;
    bool is_armed = true;
    bool was_armed = true;

    _actuators.control[0] = 0.0f;
    _actuators.control[1] = 0.0f;
    _actuators.control[2] = 0.0f;
    _actuators.control[3] = 0.0f;
    _actuators.control[4] = 1.0f;

    // opening file
    // param values to be loaded
    float max_speed_value = 0.01f;
    float max_pulse_volts_value = 0.01;
    set_parameter("PROP_MAX_SPEED", &max_speed_value);
    set_parameter("PROP_MAX_PULSE", &max_pulse_volts_value);
    // time_t t = time(NULL);
    // struct tm tm = *localtime(&t);
    // PX4_INFO("now: %d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    // PX4_INFO(filename);
    FILE *fptr;
    char filepath[62];
    sprintf(filepath,"/fs/microsd/test/%s.dat",filename);
    fptr = fopen(filepath, "w");
    fprintf(fptr, "%s", "- Swashplateless test \n");
    fprintf(fptr, "- Speed = [%.2f,%.2f], %d steps",speed_min*max_speed_value,speed_max*max_speed_value,speed_n);
    fprintf(fptr, "- Pulse = [%.2f,%.2f], %d steps",pulse_min*max_pulse_volts_value,pulse_max*max_pulse_volts_value,pulse_n);
    fprintf(fptr, "- Pulse phase = %.2f",test_phase);
    fprintf(fptr, "- Step time = %.2fs",step_time);
    fprintf(fptr, "- Plot pause time: %f seconds \n", plot_step);

    fprintf(fptr, "%s", "time \t rotation_speed \t pulse \t qa \t qx \t qy \t qz \n\n");

    int start, stop;
    int i = 0, j = -1;
    start = hrt_absolute_time();
    plot_start = start;
    stop = hrt_absolute_time();
    PX4_INFO("Starting test...");
    // PX4_INFO("i = %d/%d, j = %d/%d, time = %.2f",i+1,speed_n,j+1,pulse_n,start/1000000.0);
    PX4_INFO("Getting starting data, time = %.2f",start/1000000.0);
    float speed = 0.0;
    float pulse = 0.0;

    while(!thread_should_exit){
        // if(j > -1){ // motor stopped for first test
        //     if(speed_n == 1 || speed_max == speed_min)
        //         speed = speed_min;
        //     else
        //         speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;

        //     if(pulse_n == 1 || pulse_max == pulse_min)
        //         pulse = pulse_min;
        //     else
        //         pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
        // }
        _actuators.control[3] = speed;
        _actuators.control[0] = pulse*cos(test_phase);
        _actuators.control[1] = pulse*sin(test_phase);

        if((stop - start)/1000000.0 > step_time){
            if(j == pulse_n - 1 & i == speed_n - 1){
                thread_should_exit = true;
            } else if (j == pulse_n - 1) {
                j = 0;
                i++;
            } else {
                j++;
            }
            // PX4_INFO("Speed: (%d/%d) \t Pulse: (%d/%d)",i+1,speed_n,j+1, pulse_n);
            // speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;
            // pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;

            if(j > -1){ // motor stopped for first test
                if(speed_n == 1 || speed_max == speed_min)
                    speed = speed_min;
                else
                    speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;

                if(pulse_n == 1 || pulse_max == pulse_min)
                    pulse = pulse_min;
                else
                    pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
            }

            PX4_INFO("Speed = %f (%d/%d) \t Pulse = %f (%d/%d)",speed*max_speed_value, i+1,speed_n,pulse*max_pulse_volts_value, j+1, pulse_n);
            // speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;
            // pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
            // _actuators.control[3] = speed;
            // _actuators.control[0] = pulse*cos(test_phase*M_DEG_TO_RAD);
            // _actuators.control[1] = pulse*sin(test_phase*M_DEG_TO_RAD);
            start = hrt_absolute_time();
            // PX4_INFO("i = %d/%d, j = %d/%d, time = %.2f",i+1,speed_n,j+1,pulse_n,stop/1000000.0);
        }
        publish_actuators();
        stop = hrt_absolute_time();

        // handle errors
		int poll_ret = px4_poll(fds, 2, 150);
        if (poll_ret > 0) {
            if ((stop - plot_start)/1000 > plot_step){
                struct vehicle_attitude_s vehicle_attitude_raw;
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &vehicle_attitude_raw);
                // PX4_DEBUG("%d, %f, %f, %f, %f, %f, %f",
                //     stop,
                //     speed,
                //     pulse,
                //     vehicle_attitude_raw.q[0],
                //     vehicle_attitude_raw.q[1],
                //     vehicle_attitude_raw.q[2],
                //     vehicle_attitude_raw.q[3]);

                fprintf(fptr, "%d, %f, %f, %f, %f, %f, %f\n",
                    stop,
                    speed*max_speed_value,
                    pulse*max_pulse_volts_value,
                    vehicle_attitude_raw.q[0],
                    vehicle_attitude_raw.q[1],
                    vehicle_attitude_raw.q[2],
                    vehicle_attitude_raw.q[3]);
                plot_start = hrt_absolute_time();
            }
        }

    }


    _actuators.control[3] = 0.0;
    _actuators.control[0] = 0.0;
    _actuators.control[1] = 0.0;
    start = hrt_absolute_time();
    stop = hrt_absolute_time();
    PX4_INFO("Sending stop signal..., time = %.2f",stop/1000000.0);
    thread_should_exit = false;
    while(!thread_should_exit){
        if((stop - start)/1000000.0 > stop_signal_time)
            thread_should_exit = true;
        stop = hrt_absolute_time();
        publish_actuators();
    }
    PX4_INFO("Stop signal sent, time = %.2f",stop/1000000.0);
    PX4_INFO("Test finished, exiting.");
    fclose(fptr);
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
        // equivalent to typing:
        // iq_test start 0.0 0.0 1 0.1 0.1 2 0 0.5 0
        // iq_test start 0.1 0.1 1 0.1 0.1 2 5 0.5 5
        // iq_test start 0.4 0.4 1 0 0.8 3 115 1 20
        // iq_test start 0.2 0.4 1 0 0.8 3 115 1 20

        // iq_test start 0.4 0.4 1 0 0.8 6 115 4.5 20 2020_11_17/vel_04_pulse_0_08
        // iq_test start 0.5 0.5 1 0 0.8 6 115 4.5 20 2020_11_17/vel_05_pulse_0_08
        // iq_test start 0.6 0.6 1 0 0.8 6 115 4.5 20 2020_11_17/vel_06_pulse_0_08
        // iq_test start 0.7 0.7 1 0 0.8 6 115 4.5 20 2020_11_17/vel_07_pulse_0_08
        // iq_test start 0.8 0.8 1 0 0.8 6 115 4.5 20 2020_11_17/vel_08_pulse_0_08
        speed_min = 0.3;
        speed_max = speed_min;
        speed_n = 1;
        pulse_min = 0;
        pulse_max = 0.8;
        pulse_n = 12;
        test_phase = 0;
        step_time = 1.0;
        plot_step = 20;

        switch (argc)
        {
        case 12: strcpy(filename,argv[11]);
        case 11: plot_step = atoi(argv[10]);
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

        // param values to be loaded
        float max_speed_value = 0.01f;
        float max_pulse_volts_value = 0.01;
        set_parameter("PROP_MAX_SPEED", &max_speed_value);
        set_parameter("PROP_MAX_PULSE", &max_pulse_volts_value);
        PX4_INFO("Speed = [%.2f,%.2f], %d steps",speed_min*max_speed_value,speed_max*max_speed_value,speed_n);
        PX4_INFO("Pulse = [%.2f,%.2f], %d steps",pulse_min*max_pulse_volts_value,pulse_max*max_pulse_volts_value,pulse_n);
        PX4_INFO("Pulse phase = %.2f",test_phase);
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


// /**
//  * Load single parameter
//  */
// int set_parameter(char *param_name, float *param_value){
//     param_t param;
//     param = param_find(param_name);
//     if (param != PARAM_INVALID){
//         param_get(param, param_value);
//         PX4_INFO("%s set to %f", param_name, *param_value);
//     } else {
//         PX4_WARN("%s param invalid", param_name);
//         return -1;
//     }
//     return 0;
// }
