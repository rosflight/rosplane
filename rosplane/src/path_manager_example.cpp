#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "path_manager_example.hpp"

namespace rosplane
{

PathManagerExample::PathManagerExample()
{
  fil_state_ = FilletState::STRAIGHT;
  dub_state_ = DubinState::FIRST;

  // Declare the parameters used in this class with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  start_time_ = std::chrono::system_clock::now();

  first_ = true;
}

void PathManagerExample::manage(const Input & input, Output & output)
{
  // For readability, declare the parameters that will be used in the function here
  double R_min = params_.get_double("R_min");
  double default_altitude = params_.get_double(
    "default_altitude"); // This is the true altitude not the down position (no need for a negative)
  double default_airspeed = params_.get_double("default_airspeed");

  if (num_waypoints_ == 0) {
    auto now = std::chrono::system_clock::now();
    if (float(std::chrono::system_clock::to_time_t(now)
              - std::chrono::system_clock::to_time_t(start_time_))
        >= 10.0) {
      // TODO: Add check to see if the aircraft has been armed. If not just send the warning once before flight then on the throttle after.
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "No waypoints received, orbiting origin at " << default_altitude
                                                                               << " meters.");
      output.flag = false;            // Indicate that the path is an orbit.
      output.va_d = default_airspeed; // Set to the default_airspeed.
      output.q[0] = 0.0f;             // initialize the parameters to have a value.
      output.q[1] = 0.0f;
      output.q[2] = 0.0f;
      output.r[0] = 0.0f; // initialize the parameters to have a value.
      output.r[1] = 0.0f;
      output.r[2] = 0.0f;
      output.c[0] =
        0.0f; // Direcct the center of the orbit to the origin at the default default_altitude.
      output.c[1] = 0.0f;
      output.c[2] = -default_altitude;
      output.rho = R_min; // Make the orbit at the minimum turn radius.
      output.lamda = 1;   // Orbit in a clockwise manner.
    }
  } else if (num_waypoints_ == 1) {
    // If only a single waypoint is given, orbit it.
    output.flag = false;
    output.va_d = waypoints_[0].va_d;
    output.q[0] = 0.0f; // initialize the parameters to have a value.
    output.q[1] = 0.0f;
    output.q[2] = 0.0f;
    output.r[0] = 0.0f; // initialize the parameters to have a value.
    output.r[1] = 0.0f;
    output.r[2] = 0.0f;
    output.c[0] = waypoints_[0].w[0];
    output.c[1] = waypoints_[0].w[1];
    output.c[2] = waypoints_[0].w[2];
    output.rho = R_min;
    output.lamda =
      orbit_direction(input.pn, input.pe, input.chi, output.c[0],
                      output.c[1]); // Calculate the most conveinent orbit direction of that point.
  } else {
    if (waypoints_[idx_a_].use_chi) {
      manage_dubins(input, output);
    } else { // If the heading through the point does not matter use the default path following.
      /** Switch the following for flying directly to waypoints, or filleting corners */
      //manage_line(input, output); // TODO add ROS param for just line following or filleting?
      manage_fillet(input, output);
    }
  }
}

void PathManagerExample::manage_line(const Input & input, Output & output)
{
  // For readability, declare the parameters that will be used in the function here
  bool orbit_last = params_.get_bool("orbit_last");

  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  int idx_b;
  int idx_c;

  increment_indices(idx_a_, idx_b, idx_c, input, output);

  if (orbit_last && (idx_a_ == num_waypoints_ - 1 || idx_a_ == num_waypoints_ - 2)) {
    return;
  }

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w);
  Eigen::Vector3f w_i(waypoints_[idx_b].w);
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

  // Fill out data for straight line to the next point.
  output.flag = true;
  output.va_d = waypoints_[idx_a_].va_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  output.q[0] = q_im1(0);
  output.q[1] = q_im1(1);
  output.q[2] = q_im1(2);

  Eigen::Vector3f n_i = (q_im1 + q_i).normalized();

  // Check if the planes were aligned and then handle the normal vector correctly.
  if (n_i.isZero()) {
    n_i = q_im1;
  }

  // If the aircraft passes through the plane that bisects the angle between the waypoint lines, transition.
  if ((p - w_i).dot(n_i) > 0.0f) {
    if (idx_a_ == num_waypoints_ - 1) {
      idx_a_ = 0;
    } else {
      idx_a_++;
    }
  }
}

void PathManagerExample::manage_fillet(const Input & input, Output & output)
{
  // For readability, declare the parameters that will be used in the function here
  bool orbit_last = params_.get_bool("orbit_last");
  double R_min = params_.get_double("R_min");

  if (num_waypoints_ < 3) // Do not attempt to fillet between only 2 points.
  {
    manage_line(input, output);
    return;
  }

  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  // idx_a is the waypoint you are coming from.
  int idx_b; // Next waypoint.
  int idx_c; // Waypoint after next.

  increment_indices(idx_a_, idx_b, idx_c, input, output);

  if (orbit_last && idx_a_ == num_waypoints_ - 1) {
    return;
  }

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w); // Previous waypoint NED im1 means i-1
  Eigen::Vector3f w_i(waypoints_[idx_b].w);    // Waypoint the aircraft is headed towards.
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);  // Waypoint after leaving waypoint idx_b.

  output.va_d = waypoints_[idx_a_].va_d; // Desired airspeed of this leg of the waypoints.
  output.r[0] = w_im1(0);                // See chapter 11 of the UAV book for more information.
  output.r[1] = w_im1(1); // This is the point that is a point along the commanded path.
  output.r[2] = w_im1(2);
  // The vector pointing into the turn (vector pointing from previous waypoint to the next).
  Eigen::Vector3f q_im1 = (w_i - w_im1);
  float dist_w_im1 = q_im1.norm();
  q_im1 = q_im1.normalized();

  // The vector pointing out of the turn (vector points from next waypoint to the next next waypoint).
  Eigen::Vector3f q_i = (w_ip1 - w_i);
  float dist_w_ip1 = q_i.norm();
  q_i = q_i.normalized();

  float varrho = acosf(-q_im1.dot(q_i)); // Angle of the turn.

  // Check to see if filleting is possible for given waypoints.
  // Find closest dist to w_i
  float min_dist = std::min(dist_w_ip1, dist_w_im1);

  // Use varrho to find the distance to bisector from closest waypoint.
  float max_r = min_dist * sinf(varrho / 2.0);

  // If max_r (maximum radius possible for angle) is smaller than R_min, do line management.
  if (R_min > max_r) {
    // While in the too acute region, publish notice every 10 seconds.
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Too acute an angle, using line management. Values, max_r: " << max_r << " R_min: " << R_min);
    manage_line(input, output);
    return;
  }

  Eigen::Vector3f z;
  switch (fil_state_) {
    case FilletState::STRAIGHT: {
      output.flag = true; // Indicate flying a straight path.
      output.q[0] = q_im1(
        0); // Fly along vector into the turn the origin of the vector is r (set as previous waypoint above).
      output.q[1] = q_im1(1);
      output.q[2] = q_im1(2);
      output.c[0] = 1; // Fill rest of the data though it is not used.
      output.c[1] = 1;
      output.c[2] = 1;
      output.rho = 1;
      output.lamda = 1;
      z = w_i
        - q_im1
          * (R_min
             / tanf(
               varrho
               / 2.0)); // Point in plane where after passing through the aircraft should begin the turn.

      if ((p - z).dot(q_im1) > 0) {
        if (q_i == q_im1) // Check to see if the waypoint is directly between the next two.
        {
          if (idx_a_ == num_waypoints_ - 1)
            idx_a_ = 0;
          else
            idx_a_++;
          break;
        }
        fil_state_ = FilletState::TRANSITION; // Check to see if passed through the plane.
      }
      break;
    }
    case FilletState::TRANSITION: {
      output.flag = false; // Indicate that aircraft is following an orbit.
      output.q[0] =
        q_i(0); // Load the message with the vector that will be follwed after the orbit.
      output.q[1] = q_i(1);
      output.q[2] = q_i(2);
      Eigen::Vector3f c = w_i
        - (q_im1 - q_i).normalized()
          * (R_min / sinf(varrho / 2.0)); // Calculate the center of the orbit.
      output.c[0] = c(0);                 // Load message with the center of the orbit.
      output.c[1] = c(1);
      output.c[2] = c(2);
      output.rho = R_min; // Command the orbit radius to be the minimum acheivable.
      output.lamda = ((q_im1(0) * q_i(1) - q_im1(1) * q_i(0)) > 0
                        ? 1
                        : -1); // Find the direction to orbit the point.
      z = w_i
        + q_i
          * (R_min
             / tanf(
               varrho
               / 2.0)); // Find the point in the plane that once you pass through you should increment the indexes and follow a straight line.

      if (orbit_last && idx_a_ == num_waypoints_ - 2) {
        idx_a_++;
        fil_state_ = FilletState::STRAIGHT;
        break;
      }

      if ((p - z).dot(q_i) < 0) { // Check to see if passed through plane.
        fil_state_ = FilletState::ORBIT;
      }
      break;
    }
    case FilletState::ORBIT: {
      output.flag = false; // Indicate that aircraft is following an orbit.
      output.q[0] =
        q_i(0); // Load the message with the vector that will be follwed after the orbit.
      output.q[1] = q_i(1);
      output.q[2] = q_i(2);
      Eigen::Vector3f c = w_i
        - (q_im1 - q_i).normalized()
          * (R_min / sinf(varrho / 2.0)); // Calculate the center of the orbit.
      output.c[0] = c(0);                 // Load message with the center of the orbit.
      output.c[1] = c(1);
      output.c[2] = c(2);
      output.rho = R_min; // Command the orbit radius to be the minimum acheivable.
      output.lamda =
        ((q_im1(0) * q_i(1) - q_im1(1) * q_i(0)) > 0
           ? 1
           : -1); // Find the direction to orbit the point. TODO change this to the orbit_direction.
      z = w_i
        + q_i
          * (R_min
             / tanf(
               varrho
               / 2.0)); // Find the point in the plane that once you pass through you should increment the indexes and follow a straight line.
      if ((p - z).dot(q_i) > 0) { // Check to see if passed through plane.
        if (idx_a_ == num_waypoints_ - 1)
          idx_a_ = 0;
        else
          idx_a_++;
        fil_state_ = FilletState::STRAIGHT;
      }
      break;
    }
  }
}

void PathManagerExample::manage_dubins(const Input & input, Output & output)
{
  // For readability, declare the parameters that will be used in the function here
  double R_min = params_.get_double("R_min");

  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  output.va_d = waypoints_[idx_a_].va_d;
  output.r[0] = 0;
  output.r[1] = 0;
  output.r[2] = 0;
  output.q[0] = 0;
  output.q[1] = 0;
  output.q[2] = 0;
  output.c[0] = 0;
  output.c[1] = 0;
  output.c[2] = 0;

  switch (dub_state_) {
    case DubinState::FIRST:
      dubins_parameters(waypoints_[0], waypoints_[1], R_min);
      output.flag = false;
      output.c[0] = dubins_path_.cs(0);
      output.c[1] = dubins_path_.cs(1);
      output.c[2] = dubins_path_.cs(2);
      output.rho = dubins_path_.R;
      output.lamda = dubins_path_.lams;
      if ((p - dubins_path_.w1).dot(dubins_path_.q1) >= 0) // start in H1
      {
        dub_state_ = DubinState::BEFORE_H1_WRONG_SIDE;
      } else {
        dub_state_ = DubinState::BEFORE_H1;
      }
      break;
    case DubinState::BEFORE_H1:
      output.flag = false;
      output.c[0] = dubins_path_.cs(0);
      output.c[1] = dubins_path_.cs(1);
      output.c[2] = dubins_path_.cs(2);
      output.rho = dubins_path_.R;
      output.lamda = dubins_path_.lams;
      if ((p - dubins_path_.w1).dot(dubins_path_.q1) >= 0) // entering H1
      {
        dub_state_ = DubinState::STRAIGHT;
      }
      break;
    case DubinState::BEFORE_H1_WRONG_SIDE:
      output.flag = false;
      output.c[0] = dubins_path_.cs(0);
      output.c[1] = dubins_path_.cs(1);
      output.c[2] = dubins_path_.cs(2);
      output.rho = dubins_path_.R;
      output.lamda = dubins_path_.lams;
      if ((p - dubins_path_.w1).dot(dubins_path_.q1) < 0) // exit H1
      {
        dub_state_ = DubinState::BEFORE_H1;
      }
      break;
    case DubinState::STRAIGHT:
      output.flag = true;
      output.r[0] = dubins_path_.w1(0);
      output.r[1] = dubins_path_.w1(1);
      output.r[2] = dubins_path_.w1(2);
      // output.r[0] = dubinspath_.z1(0);
      // output.r[1] = dubinspath_.z1(1);
      // output.r[2] = dubinspath_.z1(2);
      output.q[0] = dubins_path_.q1(0);
      output.q[1] = dubins_path_.q1(1);
      output.q[2] = dubins_path_.q1(2);
      output.rho = 1;
      output.lamda = 1;
      if ((p - dubins_path_.w2).dot(dubins_path_.q1) >= 0) // entering H2
      {
        if ((p - dubins_path_.w3).dot(dubins_path_.q3) >= 0) // start in H3
        {
          dub_state_ = DubinState::BEFORE_H3_WRONG_SIDE;
        } else {
          dub_state_ = DubinState::BEFORE_H3;
        }
      }
      break;
    case DubinState::BEFORE_H3:
      output.flag = false;
      output.c[0] = dubins_path_.ce(0);
      output.c[1] = dubins_path_.ce(1);
      output.c[2] = dubins_path_.ce(2);
      output.rho = dubins_path_.R;
      output.lamda = dubins_path_.lame;
      if ((p - dubins_path_.w3).dot(dubins_path_.q3) >= 0) // entering H3
      {
        // increase the waypoint pointer
        int idx_b;
        if (idx_a_ == num_waypoints_ - 1) {
          idx_a_ = 0;
          idx_b = 1;
        } else if (idx_a_ == num_waypoints_ - 2) {
          idx_a_++;
          idx_b = 0;
        } else {
          idx_a_++;
          idx_b = idx_a_ + 1;

          if (first_) {
            first_ = false;
            waypoints_.erase(waypoints_.begin());
            num_waypoints_--;
            idx_a_--;
            idx_b = idx_a_ + 1;
          }
        }

        // plan new Dubin's path to next waypoint configuration
        dubins_parameters(waypoints_[idx_a_], waypoints_[idx_b], R_min);

        //start new path
        if ((p - dubins_path_.w1).dot(dubins_path_.q1) >= 0) // start in H1
        {
          dub_state_ = DubinState::BEFORE_H1_WRONG_SIDE;
        } else {
          dub_state_ = DubinState::BEFORE_H1;
        }
      }
      break;
    case DubinState::BEFORE_H3_WRONG_SIDE:
      output.flag = false;
      output.c[0] = dubins_path_.ce(0);
      output.c[1] = dubins_path_.ce(1);
      output.c[2] = dubins_path_.ce(2);
      output.rho = dubins_path_.R;
      output.lamda = dubins_path_.lame;
      if ((p - dubins_path_.w3).dot(dubins_path_.q3) < 0) // exit H3
      {
        dub_state_ = DubinState::BEFORE_H1;
      }
      break;
  }
}

Eigen::Matrix3f PathManagerExample::rotz(float theta)
{
  Eigen::Matrix3f R;
  R << cosf(theta), -sinf(theta), 0, sinf(theta), cosf(theta), 0, 0, 0, 1;

  return R;
}

float PathManagerExample::mo(float in)
{
  float val;
  if (in > 0)
    val = fmod(in, 2.0 * M_PI_F);
  else {
    float n = floorf(in / 2.0 / M_PI_F);
    val = in - n * 2.0 * M_PI_F;
  }
  return val;
}

void PathManagerExample::dubins_parameters(const Waypoint start_node, const Waypoint end_node,
                                           float R)
{
  float ell = sqrtf((start_node.w[0] - end_node.w[0]) * (start_node.w[0] - end_node.w[0])
                    + (start_node.w[1] - end_node.w[1]) * (start_node.w[1] - end_node.w[1]));
  if (ell < 2.0 * R) {
    RCLCPP_ERROR(this->get_logger(), "The distance between nodes must be larger than 2R.");

  } else {
    dubins_path_.ps(0) = start_node.w[0];
    dubins_path_.ps(1) = start_node.w[1];
    dubins_path_.ps(2) = start_node.w[2];
    dubins_path_.chis = start_node.chi_d;
    dubins_path_.pe(0) = end_node.w[0];
    dubins_path_.pe(1) = end_node.w[1];
    dubins_path_.pe(2) = end_node.w[2];
    dubins_path_.chie = end_node.chi_d;

    Eigen::Vector3f crs = dubins_path_.ps;
    crs(0) +=
      R * (cosf(M_PI_2_F) * cosf(dubins_path_.chis) - sinf(M_PI_2_F) * sinf(dubins_path_.chis));
    crs(1) +=
      R * (sinf(M_PI_2_F) * cosf(dubins_path_.chis) + cosf(M_PI_2_F) * sinf(dubins_path_.chis));
    Eigen::Vector3f cls = dubins_path_.ps;
    cls(0) +=
      R * (cosf(-M_PI_2_F) * cosf(dubins_path_.chis) - sinf(-M_PI_2_F) * sinf(dubins_path_.chis));
    cls(1) +=
      R * (sinf(-M_PI_2_F) * cosf(dubins_path_.chis) + cosf(-M_PI_2_F) * sinf(dubins_path_.chis));
    Eigen::Vector3f cre = dubins_path_.pe;
    cre(0) +=
      R * (cosf(M_PI_2_F) * cosf(dubins_path_.chie) - sinf(M_PI_2_F) * sinf(dubins_path_.chie));
    cre(1) +=
      R * (sinf(M_PI_2_F) * cosf(dubins_path_.chie) + cosf(M_PI_2_F) * sinf(dubins_path_.chie));
    Eigen::Vector3f cle = dubins_path_.pe;
    cle(0) +=
      R * (cosf(-M_PI_2_F) * cosf(dubins_path_.chie) - sinf(-M_PI_2_F) * sinf(dubins_path_.chie));
    cle(1) +=
      R * (sinf(-M_PI_2_F) * cosf(dubins_path_.chie) + cosf(-M_PI_2_F) * sinf(dubins_path_.chie));

    float theta, theta2;
    // compute L1
    theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
    float L1 = (crs - cre).norm()
      + R * mo(2.0 * M_PI_F + mo(theta - M_PI_2_F) - mo(dubins_path_.chis - M_PI_2_F))
      + R * mo(2.0 * M_PI_F + mo(dubins_path_.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

    // compute L2
    ell = (cle - crs).norm();
    theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
    float L2;
    if (2.0 * R > ell)
      L2 = 9999.0f;
    else {
      theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
      L2 = sqrtf(ell * ell - 4.0 * R * R)
        + R * mo(2.0 * M_PI_F + mo(theta2) - mo(dubins_path_.chis - M_PI_2_F))
        + R * mo(2.0 * M_PI_F + mo(theta2 + M_PI_F) - mo(dubins_path_.chie + M_PI_2_F));
    }

    // compute L3
    ell = (cre - cls).norm();
    theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
    float L3;
    if (2.0 * R > ell)
      L3 = 9999.0f;
    else {
      theta2 = acosf(2.0 * R / ell);
      L3 = sqrtf(ell * ell - 4 * R * R)
        + R * mo(2.0 * M_PI_F + mo(dubins_path_.chis + M_PI_2_F) - mo(theta + theta2))
        + R * mo(2.0 * M_PI_F + mo(dubins_path_.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
    }

    // compute L4
    theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
    float L4 = (cls - cle).norm()
      + R * mo(2.0 * M_PI_F + mo(dubins_path_.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
      + R * mo(2.0 * M_PI_F + mo(theta + M_PI_2_F) - mo(dubins_path_.chie + M_PI_2_F));

    // L is the minimum distance
    int idx = 1;
    dubins_path_.L = L1;
    if (L2 < dubins_path_.L) {
      dubins_path_.L = L2;
      idx = 2;
    }
    if (L3 < dubins_path_.L) {
      dubins_path_.L = L3;
      idx = 3;
    }
    if (L4 < dubins_path_.L) {
      dubins_path_.L = L4;
      idx = 4;
    }

    Eigen::Vector3f e1;
    //        e1.zero();
    e1(0) = 1;
    e1(1) = 0;
    e1(2) = 0;
    switch (idx) {
      case 1:
        dubins_path_.cs = crs;
        dubins_path_.lams = 1;
        dubins_path_.ce = cre;
        dubins_path_.lame = 1;
        dubins_path_.q1 = (cre - crs).normalized();
        dubins_path_.w1 = dubins_path_.cs + (rotz(-M_PI_2_F) * dubins_path_.q1) * R;
        dubins_path_.w2 = dubins_path_.ce + (rotz(-M_PI_2_F) * dubins_path_.q1) * R;
        break;
      case 2:
        dubins_path_.cs = crs;
        dubins_path_.lams = 1;
        dubins_path_.ce = cle;
        dubins_path_.lame = -1;
        ell = (cle - crs).norm();
        theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
        theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
        dubins_path_.q1 = rotz(theta2 + M_PI_2_F) * e1;
        dubins_path_.w1 = dubins_path_.cs + (rotz(theta2) * e1) * R;
        dubins_path_.w2 = dubins_path_.ce + (rotz(theta2 + M_PI_F) * e1) * R;
        break;
      case 3:
        dubins_path_.cs = cls;
        dubins_path_.lams = -1;
        dubins_path_.ce = cre;
        dubins_path_.lame = 1;
        ell = (cre - cls).norm();
        theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
        theta2 = acosf(2.0 * R / ell);
        dubins_path_.q1 = rotz(theta + theta2 - M_PI_2_F) * e1;
        dubins_path_.w1 = dubins_path_.cs + (rotz(theta + theta2) * e1) * R;
        dubins_path_.w2 = dubins_path_.ce + (rotz(theta + theta2 - M_PI_F) * e1) * R;
        break;
      case 4:
        dubins_path_.cs = cls;
        dubins_path_.lams = -1;
        dubins_path_.ce = cle;
        dubins_path_.lame = -1;
        dubins_path_.q1 = (cle - cls).normalized();
        dubins_path_.w1 = dubins_path_.cs + (rotz(M_PI_2_F) * dubins_path_.q1) * R;
        dubins_path_.w2 = dubins_path_.ce + (rotz(M_PI_2_F) * dubins_path_.q1) * R;
        break;
    }
    dubins_path_.w3 = dubins_path_.pe;
    dubins_path_.q3 = rotz(dubins_path_.chie) * e1;
    dubins_path_.R = R;
  }
}

void PathManagerExample::declare_parameters() { params_.declare_bool("orbit_last", false); }

int PathManagerExample::orbit_direction(float pn, float pe, float chi, float c_n, float c_e)
{
  if (orbit_dir_ != 0) {
    return orbit_dir_;
  }

  Eigen::Vector3f pos;
  pos << pn, pe, 0.0;

  Eigen::Vector3f center;
  center << c_n, c_e, 0.0;

  Eigen::Vector3f d = pos - center;

  Eigen::Vector3f course;
  course << sinf(chi), cosf(chi), 0.0;

  if (d.cross(course)(2) >= 0) {
    orbit_dir_ = 1;
    return 1;
  }

  orbit_dir_ = -1;
  return -1;
}

void PathManagerExample::increment_indices(int & idx_a, int & idx_b, int & idx_c,
                                           const Input & input, Output & output)
{

  bool orbit_last = params_.get_bool("orbit_last");
  double R_min = params_.get_double("R_min");

  if (temp_waypoint_ && idx_a_ == 1) {
    waypoints_.erase(waypoints_.begin());
    num_waypoints_--;
    idx_a_ = 0;
    idx_b = 1;
    idx_c = 2;
    temp_waypoint_ = false;
    return;
  }

  if (idx_a == num_waypoints_ - 1) { // The logic for if it is the last waypoint.

    // If it is the last waypoint, and we orbit the last waypoint, construct the command.
    if (orbit_last) {
      output.flag = false;
      output.va_d = waypoints_[idx_a_].va_d;
      output.c[0] = waypoints_[idx_a_].w[0];
      output.c[1] = waypoints_[idx_a_].w[1];
      output.c[2] = waypoints_[idx_a_].w[2];
      output.r[0] = 0.0;
      output.r[1] = 0.0;
      output.r[2] = 0.0;
      output.q[0] = 0.0;
      output.q[1] = 0.0;
      output.q[2] = 0.0;
      output.rho = R_min;
      output.lamda = orbit_direction(
        input.pn, input.pe, input.chi, output.c[0],
        output.c[1]); // Calculate the most conveinent orbit direction of that point.
      idx_b = 0;      // reset the path and loop the waypoints again.
      idx_c = 1;
      return;
    }

    idx_b = 0; // reset the path and loop the waypoints again.
    idx_c = 1;
  } else if (
    idx_a
    == num_waypoints_
      - 2) { // If the second to last waypoint, appropriately handle the wrapping of waypoints.
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  } else { // Increment the indices of the waypoints.
    idx_b = idx_a + 1;
    idx_c = idx_b + 1;
  }
}

} // namespace rosplane
