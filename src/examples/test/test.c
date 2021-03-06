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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/input_rc.h>

#define clrscr() PX4_INFO("\e[1;1H\e[2J")


__EXPORT int test_main(int argc, char *argv[]);

int test_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    // /* subscribe to sensor_combined topic */
    // int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    // /* limit the update rate to 5 Hz */
    // orb_set_interval(sensor_sub_fd, 200);

    // /* advertise attitude topic */
    // struct vehicle_attitude_s att;
    // memset(&att, 0, sizeof(att));
    // orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

    // /* one could wait for multiple topics with this technique, just using one here */
    // px4_pollfd_struct_t fds[] = {
    //     { .fd = sensor_sub_fd,   .events = POLLIN },
    //     /* there could be more file descriptors here, in the form like:
    //      * { .fd = other_sub_fd,   .events = POLLIN },
    //      */int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    // };

    /* subscribe to sensor_combined topic */
    int actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	int actuator_arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    int input_rc_sub_fd = orb_subscribe(ORB_ID(input_rc));
    int vehicle_status_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
    int vehicle_control_mode_sub_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
    int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 5 Hz */
    orb_set_interval(actuator_ctrl_sub_fd, 100);
    orb_set_interval(actuator_ctrl_manual_sub_fd, 100);
    orb_set_interval(actuator_arm_sub_fd, 100);
    orb_set_interval(input_rc_sub_fd, 100);
    orb_set_interval(vehicle_status_sub_fd, 100);
    orb_set_interval(vehicle_control_mode_sub_fd, 100);
    orb_set_interval(vehicle_attitude_sub_fd, 100);
    orb_set_interval(sensor_combined_sub_fd, 100);

    // /* advertise attitude topic */
    // struct vehicle_attitude_s att;
    // memset(&att, 0, sizeof(att));status
    // orb_advert_t att_pub = orb_advertise(ORB_ID(actuator_controls_0), &att);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = actuator_ctrl_sub_fd,   .events = POLLIN },
        { .fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN },
        { .fd = actuator_arm_sub_fd,   .events = POLLIN },
        { .fd = input_rc_sub_fd,   .events = POLLIN },
        { .fd = vehicle_status_sub_fd,   .events = POLLIN },
        { .fd = vehicle_control_mode_sub_fd,   .events = POLLIN },
        { .fd = vehicle_attitude_sub_fd,   .events = POLLIN },
        { .fd = sensor_combined_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;
    //float velocity, x_roll, y_pitch, z_yaw, amplitude, phase;

    while(true){
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct actuator_controls_s actuator_raw;
                orb_copy(ORB_ID(actuator_controls_0), actuator_ctrl_sub_fd, &actuator_raw);
                struct actuator_controls_s actuator_manual_raw;
                orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_manual_raw);
                struct actuator_armed_s actuator_arm_raw;
                orb_copy(ORB_ID(actuator_armed), actuator_arm_sub_fd, &actuator_arm_raw);
                struct input_rc_s rc_raw;
                orb_copy(ORB_ID(input_rc), input_rc_sub_fd, &rc_raw);
                struct vehicle_status_s vehicle_status_raw;
                orb_copy(ORB_ID(vehicle_status), vehicle_status_sub_fd, &vehicle_status_raw);
                struct vehicle_control_mode_s vehicle_control_mode_raw;
                orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub_fd, &vehicle_control_mode_raw);
                struct vehicle_attitude_s vehicle_attitude_raw;
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &vehicle_attitude_raw);
                struct sensor_combined_s sensor_combined_raw;
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined_raw);


                double acc_x = sensor_combined_raw.accelerometer_m_s2[0];
                double acc_y = sensor_combined_raw.accelerometer_m_s2[1];
                double acc_z = sensor_combined_raw.accelerometer_m_s2[2];
                double phase = atan2(acc_y,acc_x);

                clrscr();
                PX4_INFO("|\n\
                    is_armed = %i\n\
                    control_manual = %i\n\
                    control_auto = %i\n\
                    control_position = %i\n\
                    rc_mode = %i\n\
                    acc_x = %f\n\
                    acc_y = %f\n\
                    acc_z = %f\n\
                    phase = %f",
                    (bool)actuator_arm_raw.armed,
                    (bool)vehicle_control_mode_raw.flag_control_manual_enabled,
                    (bool)vehicle_control_mode_raw.flag_control_auto_enabled,
                    (bool)vehicle_control_mode_raw.flag_control_position_enabled,
                    (bool)vehicle_status_raw.rc_input_mode,
                    acc_x,
                    acc_y,
                    acc_z,
                    180 - phase*M_RAD_TO_DEG
                    );

                /* set att and publish this information for other apps
                 the following does not have any meaning, it's just an example
                */
                // att.q[0] = raw.accelerometer_m_s2[0];
                // att.q[1] = raw.accelerometer_m_s2[1];
                // att.q[2] = raw.accelerometer_m_s2[2];

                // orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
            }

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("exiting");

    return 0;
}
