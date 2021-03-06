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
#include <parameters/param.h>
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


__EXPORT int Attitude_Acc_main(int argc, char *argv[]);

int Attitude_Acc_main(int argc, char *argv[])
{
    // load params
    param_t motor_phase_up_param;
    // float motor_phase_up_value = 95.5*M_DEG_TO_RAD;
    motor_phase_up_param = param_find("MOTOR_PHASE_UP");

    if (motor_phase_up_param == PARAM_INVALID) {
      PX4_WARN("MOTOR_PHASE_UP param invalid");
    }
    else
    {
        int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
        orb_set_interval(sensor_combined_sub_fd, 50);
        int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
        orb_set_interval(actuator_ctrl_manual_sub_fd, 50);
        px4_pollfd_struct_t fds[] = {
            { .fd = sensor_combined_sub_fd,   .events = POLLIN },
            { .fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN },
        };
        int n = 0, error_counter = 0;
        double ax0 = 0, ay0 = 0, az0 = 0;
        double ax, ay, az, phase, inclination;
        double gx, gy, gz;
        double q0,qx,qy,qz;

        while(true)
        {
            int poll_ret = px4_poll(fds, 2, 100);

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

                struct actuator_controls_s actuator_raw;
                orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
                double x_roll    = actuator_raw.control[0];
                double y_pitch   = actuator_raw.control[1];
                double phase = atan2(x_roll,y_pitch);

                struct sensor_combined_s sensor_combined_raw;
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined_raw);
                gx = sensor_combined_raw.gyro_rad[0];
                gy = sensor_combined_raw.gyro_rad[1];
                gz = sensor_combined_raw.gyro_rad[2];
                double omega = sqrt(gx*gx + gy*gy + gz*gz);
                ax = sensor_combined_raw.accelerometer_m_s2[0];
                ay = sensor_combined_raw.accelerometer_m_s2[1];
                az = sensor_combined_raw.accelerometer_m_s2[2];
                double norm = sqrt(ax*ax + ay*ay + az*az);
                ax = ax/norm;
                ay = ay/norm;
                az = az/norm;

                if(n == 0)
                {
                    ax0 = ax;
                    ay0 = ay;
                    az0 = 1-az;
                    n++;
                }
                az = 1-az;
                q0 = az*az0 + ax*ax0 + ay*ay0;
                qx = -ay0*az + ay*az0;
                qy =  ax0*az - ax*az0;
                qz = ax0*ay - ax*ay0;
                // phase = atan2(-ax,+acc_y);
                // double phase2 = atan2(acc_y,ax);
                double direction = atan2(-qx,qy);
                inclination = 2*atan2(sqrt(qx*qx + qy*qy + qz*qz),q0);
                // double test_array[4] = {3,3,3,3};
                // normalize(test_array, 4);
                PX4_INFO("direction = %.2f | phase = %.2f | inclination = %.2f  | omega = %.2f |", direction * M_RAD_TO_DEG, phase * M_RAD_TO_DEG, inclination * M_RAD_TO_DEG, omega * M_RAD_TO_DEG);
                // PX4_INFO("norm = %f", get_norm(test_array,4));

            }
        }

        // param_set(motor_phase_up_param, &motor_phase_up_value);
        // PX4_INFO("Motor phase set to: %f",(double)motor_phase_up_value * M_RAD_TO_DEG);
    }

    return 0;
}
