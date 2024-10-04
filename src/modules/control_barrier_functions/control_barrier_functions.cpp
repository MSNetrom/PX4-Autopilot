/****************************************************************************
 *
 *   Copyright (c) 2013-2024 PX4 Development Team. All rights reserved.
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
 * @file control_barrier_functions.cpp
 * Control Barrier Functions for depth sensor
 *
 * @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
 */

#include "control_barrier_functions.hpp"

extern "C" __EXPORT int control_barrier_functions_main(int argc, char *argv[])
{
	PX4_INFO("Hello from control_barrier_functions_main");

	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int local_position_subscribe_file_descriptor = orb_subscribe(ORB_ID(vehicle_local_position));

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = local_position_subscribe_file_descriptor,   .events = POLLIN },
	};

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub_filedescr = orb_advertise(ORB_ID(vehicle_attitude), &att);

	int error_counter = 0;
	for (int i = 0; i < 10000; i++) {

	/* wait for sensor update of 2 file descriptor for 1000 ms (1 second) */
	int poll_ret = px4_poll(fds, 2, 3000);

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

		if (fds[1].revents & POLLIN) {
			/* obtained data for the second file descriptor */
			struct vehicle_local_position_s local_position;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(vehicle_local_position), local_position_subscribe_file_descriptor, &local_position);
			//PX4_INFO("Local position setpoint:\t%8.4f",
						//(double)local_position.z);
		}
		if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct sensor_combined_s raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			//PX4_INFO("Accelerometer:\t%8.4f",
						//(double)raw.accelerometer_m_s2[0]);
		}

	}
	att.q[0] = 0.5;
	att.q[1] = 0.0;
	att.q[2] = 0.3;
	orb_publish(ORB_ID(vehicle_attitude), att_pub_filedescr, &att);

}
	return 0;
}
