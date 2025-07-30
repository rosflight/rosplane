/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Ian Reid, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file path_follower_lines_orbits.cpp
 * 
 * Implements vector field path following as described in Small Unmanned Aircraft by Tim McLain and Randy Beard
 * 
 * @author Ian Reid <ian.young.reid@gmail.com>
*/

#include "path_follower/path_follower_lines_orbits.hpp"

namespace rosplane
{

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

PathFollowerLinesOrbits::PathFollowerLinesOrbits() {}

void PathFollowerLinesOrbits::follow(const Input & input, Output & output)
{
  // For readability, declare parameters that will be used in the function here
  double k_path = params_.get_double("k_path");
  double k_orbit = params_.get_double("k_orbit");
  double chi_infty = params_.get_double("chi_infty");
  double gravity = params_.get_double("gravity");

  // If path_type is a line, follow straight line path specified by r and q
  // Otherwise, follow an orbit path specified by c_orbit, rho_orbit, and lam_orbit
  if (input.p_type == PathType::LINE) {
    // compute wrapped version of the path angle
    float chi_q = atan2f(input.q_path[1], input.q_path[0]);

    chi_q = wrap_within_180(input.chi, chi_q);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "input.chi: " << input.chi);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "chi_q: " << chi_q);

    float path_error =
      -sinf(chi_q) * (input.pn - input.r_path[0]) + cosf(chi_q) * (input.pe - input.r_path[1]);

    // heading command
    output.chi_c = chi_q - chi_infty * 2 / M_PI * atanf(k_path * path_error);

    // desired altitude
    float h_d = -input.r_path[2]
      - sqrtf(powf((input.r_path[0] - input.pn), 2) + powf((input.r_path[1] - input.pe), 2))
        * (input.q_path[2]) / sqrtf(powf(input.q_path[0], 2) + powf(input.q_path[1], 2));

    // commanded altitude is desired altitude
    output.h_c = h_d;
    output.phi_ff = 0.0;
  } else {
    float d = sqrtf(powf((input.pn - input.c_orbit[0]), 2)
                    + powf((input.pe - input.c_orbit[1]),
                           2)); // distance from orbit center

    // compute wrapped version of angular position on orbit
    float varphi = atan2f(input.pe - input.c_orbit[1], input.pn - input.c_orbit[0]);

    varphi = wrap_within_180(input.chi, varphi);

    //compute orbit error
    float norm_orbit_error = (d - input.rho_orbit) / input.rho_orbit;
    output.chi_c = varphi + input.lam_orbit * (M_PI / 2.0 + atanf(k_orbit * norm_orbit_error));

    // commanded altitude is the height of the orbit
    float h_d = -input.c_orbit[2];
    output.h_c = h_d;
    output.phi_ff = input.lam_orbit
      * std::atan(pow(input.va, 2) / (gravity * input.rho_orbit * std::cos(input.chi - input.psi)));

    output.chi_c = wrap_within_180(0.0, output.chi_c);
  }

  output.va_c = input.va_d;
}

} // namespace rosplane
