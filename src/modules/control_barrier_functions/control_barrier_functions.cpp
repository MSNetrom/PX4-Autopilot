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
	// We assume that all states and position are given in the same frame, and that the z-axis is pointing upwards
	// from the ground.
	// The output of the control barrier function is in the frame of the input states.

	// Define the position of the obstacle. (We assume that these are given to us somehow)
	math::Vector3f quad_pos(1.0f, 2.0f, 3.0f);
	math::Vector3f quad_vel(3.0f, 4.0f, 5.0f);
	math::Vector3f obstacle_pos(1.0f, 2.0f, 3.0f);
	float obstacle_radius = 5.0f;

	// We also assumed that a wished control input u_ref (acceleration in the given frame) is given to us
	math::Vector3f u_ref(1.0f, 2.0f, 3.0f);

	// We assume that the tuning constants p1 and p2 are given to us. These decide how fast the dynamics are allowed to be.
	// These must be positive.
	float p1 = 5.0f;
	float p2 = 4.0f;

	// We will model the obstacle as a cylinder and effectively ignore the z-coordinate
	math::Vector2f quad_pos_2d = quad_pos.xy();
	math::Vector2f quad_vel_2d = quad_vel.xy();
    	math::Vector2f obstacle_pos_2d = obstacle_pos.xy();
	math::Vector2f u_ref_2d = u_ref.xy();

	// Let's calculate some helper variables
	math::Vector2f diff = quad_pos_2d - obstacle_pos_2d;
	float h = diff.dot(diff) - obstacle_radius * obstacle_radius;
	float h_dot = 2.0f * diff.dot(quad_vel_2d);
	float Lf_h_dot = 2.0f * quad_vel_2d.dot(quad_vel_2d)
	math::Vector2f Lg_h_dot = 2.0f * diff

	// Let's define alpha and beta, for the optimization constraint a^Tu >= b
	math::Vector2f alpha = Lg_h_dot
	float beta = -(Lf_h_dot + (p1 + p2) * h_dot + p1 * p2 * h)

	// Let's solve this analytically
	float k = alpha.dot(u_ref_2d) / (alpha.dot(alpha))
	math::Vector2f u_safe_2d = max(beta / alpha.dot(alpha), k) * alpha + u_ref_2d - k * alpha
	math::Vector3f u_safe(u_safe_2d.x, u_safe_2d.y, u_ref.z)

	// Now we can send u_safe to pixhawk somehow

	return 0;
}
