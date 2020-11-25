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
#include <uORB/topics/actuator_controls.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <time.h>

extern "C" __EXPORT int actuator_controls_listen_main(int argc, char *argv[]);
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
bool actuators_thread_running = false;
bool actuators_thread_should_exit = false;

int actuators_time = 200;
//char filename[40] = "test";
// float q[4], acc[4], gyro[4];

//int actuator_ctrl_sub_fd;
//int actuator_ctrl_manual_sub_fd;

int actuator_controls_listen_thread_main(int argc, char *argv[])
{
    // Load parameters, set UART comm to motor, set ORB topics
    int actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
    orb_set_interval(actuator_ctrl_sub_fd, 1);
    orb_set_interval(actuator_ctrl_manual_sub_fd, 1);
    px4_pollfd_struct_t fds[] = {
        {.fd = actuator_ctrl_sub_fd,   .events = POLLIN},
        {.fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN}};

	// initialize thread variables
    int error_counter = 0;
    actuators_thread_running = true;

    int start = hrt_absolute_time();
    int stop = start;

	// main while loop for this thread
	while(!actuators_thread_should_exit)
	{
        stop = hrt_absolute_time();
        // handle errors
		int poll_ret = px4_poll(fds, 2, 150);
		// /* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a %ims", 150);

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0)
			{
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;

		} else if ((stop-start)/1000.0 > actuators_time) {

            struct actuator_controls_s actuator_raw, actuator_manual_raw;

		    if (fds[0].revents & POLLIN){
                orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_sub_fd, &actuator_raw);
            }

            if (fds[1].revents & POLLIN){
                // Get the actuator armed data
                orb_copy(ORB_ID(actuator_controls_0), actuator_ctrl_manual_sub_fd, &actuator_manual_raw);
            }

            PX4_INFO("\ncontrol_0 =\t[%f\t%f\t%f\t%f]\ncontrol_3 =\t[%f\t%f\t%f\t%f]",
                (double)actuator_raw.control[0],
                (double)actuator_raw.control[1],
                (double)actuator_raw.control[2],
                (double)actuator_raw.control[3],
                (double)actuator_manual_raw.control[0],
                (double)actuator_manual_raw.control[1],
                (double)actuator_manual_raw.control[2],
                (double)actuator_manual_raw.control[3]);
            start = stop;
        }
	}
	PX4_INFO("exiting");
    actuators_thread_running = false;

    fflush(stdout);
	return 0;
}

int actuator_controls_listen_main(int argc, char *argv[]){

    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (actuators_thread_running) {
            PX4_INFO("already running\n");
            /* this is not an error */
            return 0;
        }

        switch (argc)
        {
        case 3: actuators_time = atoi(argv[2]);
        case 2: break;
        // case 1: break;
        default:
            PX4_ERR("usage: actuator_controls_listen [start/stop] (timestep)");
            return -1;
        }

        actuators_thread_should_exit = false;

        daemon_task = px4_task_spawn_cmd("actuator_controls_listen",
                SCHED_DEFAULT,
                SCHED_PRIORITY_DEFAULT -10,
                2000,
                actuator_controls_listen_thread_main,
                //(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                //  (argv) ? (char *const *)&uart_name1 : (char *const *)NULL);
                (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!actuators_thread_running) {
            PX4_INFO("already stopped\n");
            /* this is not an error */
        } else {
            PX4_INFO("stopping");
            actuators_thread_should_exit = true;
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
