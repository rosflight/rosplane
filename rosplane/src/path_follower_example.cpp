#include "path_follower_example.hpp"
#include <rclcpp/logging.hpp>

namespace rosplane
{

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

path_follower_example::path_follower_example() {}

void path_follower_example::follow(const input_s & input,
                                   output_s & output)
{
  // For readability, declare parameters that will be used in the function here
  double k_path = params.get_double("k_path");
  double k_orbit = params.get_double("k_orbit");
  double chi_infty = params.get_double("chi_infty");

  if (input.p_type == path_type::Line) // follow straight line path specified by r and q
  {
    // compute wrapped version of the path angle
    float chi_q = atan2f(input.q_path[1], input.q_path[0]);

    chi_q = wrap_within_180(input.chi, chi_q);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "input.chi: " << input.chi);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "chi_q: " << chi_q);

    float path_error =
      -sinf(chi_q) * (input.pn - input.r_path[0]) + cosf(chi_q) * (input.pe - input.r_path[1]);

    // RCLCPP_INFO_STREAM(this->get_logger(), "k_path: " << k_path);

    // heading command
    output.chi_c = chi_q - chi_infty * 2 / M_PI * atanf(k_path * path_error);

    // desired altitude
    float h_d = -input.r_path[2]
      - sqrtf(powf((input.r_path[0] - input.pn), 2) + powf((input.r_path[1] - input.pe), 2))
        * (input.q_path[2]) / sqrtf(powf(input.q_path[0], 2) + powf(input.q_path[1], 2));
    // commanded altitude is desired altitude
    output.h_c = h_d;
    output.phi_ff = 0.0;
  } else // follow an orbit path specified by c_orbit, rho_orbit, and lam_orbit
  {
    float d = sqrtf(powf((input.pn - input.c_orbit[0]), 2)
                    + powf((input.pe - input.c_orbit[1]),
                           2)); // distance from orbit center
    // compute wrapped version of angular position on orbit
    float varphi = atan2f(input.pe - input.c_orbit[1], input.pn - input.c_orbit[0]);

    varphi = wrap_within_180(input.chi, varphi);

    //compute orbit error
    float norm_orbit_error = (d - input.rho_orbit) / input.rho_orbit;
    output.chi_c =
      varphi + input.lam_orbit * (M_PI / 2.0 + atanf(k_orbit * norm_orbit_error));

    // commanded altitude is the height of the orbit
    float h_d = -input.c_orbit[2];
    output.h_c = h_d;
    output.phi_ff = input.lam_orbit
      * std::atan(pow(input.Va, 2) / (9.81 * input.rho_orbit * std::cos(input.chi - input.psi)));

    output.chi_c = wrap_within_180(0.0, output.chi_c);
  }
  output.Va_c = input.Va_d;
}

} // namespace rosplane
