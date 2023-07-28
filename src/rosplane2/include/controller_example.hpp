#ifndef CONTROLLER_EXAMPLE_H
#define CONTROLLER_EXAMPLE_H

#include "controller_base.hpp"

namespace rosplane2
{

class controller_example : public controller_base
{
public:
  /**
   * Constructor to initialize node.
   */
  controller_example();

private:

  /**
   * The control algorithm for the autopilot.
   * @param params The parameters that define the algorithm such as control gains.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);

  /**
   * The current part of the control algorithm based on altitude.
   */
  alt_zones current_zone;

  /**
   * The control loop for moving to and holding a commanded course.
   * @param chi_c The commanded course angle.
   * @param chi The current course angle.
   * @param phi_ff The roll angle feedforward term. This allows for faster convergence.   // TODO add reference to book?
   * @param r The yaw rate taken from the gyro.
   * @param params The parameters for the control algorithm, including the control gains.
   * @param Ts The sampling period.
   * @return The commanded roll angle, to achieve the course angle.
   */
  float course_hold(float chi_c, float chi, float phi_ff, float r, const struct params_s &params, float Ts);

  /**
   * The difference between the commanded course angle and the current course angle.
   */
  float c_error_;

  /**
   * The integral of the error in course angle.
   */
  float c_integrator_;

  /**
   * The control loop for moving to and holding a commanded roll angle.
   * @param phi_c The commanded roll angle.
   * @param phi The current roll angle.
   * @param p The roll rate taken from the gyro.
   * @param params The parameters for the control algorithm, including the control gains and max deflection.
   * @param Ts The sampling period.
   * @return The aileron deflection in radians required to achieve the commanded roll angle.
   */
  float roll_hold(float phi_c, float phi, float p, const struct params_s &params, float Ts);

  /**
   * The difference between the commanded roll angle and the current roll angle.
   */
  float r_error_;

  /**
   * The integral of the error in roll angle.
   */
  float r_integrator;

  /**
   * The control loop for moving to and holding a commanded pitch angle.
   * @param theta_c The commanded pitch angle.
   * @param theta The current pitch angle.
   * @param q The pitch rate taken from the gyro.
   * @param params The parameters for the control algorithm, including the control gains and max deflection.
   * @param Ts The sampling period
   * @return The elevator deflection in radians required to achieve the commanded pitch.
   */
  float pitch_hold(float theta_c, float theta, float q, const struct params_s &params, float Ts);

  /**
   * The difference between the commanded pitch angle and the current pitch angle.
   */
  float p_error_;

  /**
   * The integral of the error in pitch angle.
   */
  float p_integrator_;

  /**
   * The control loop that calculates the commanded pitch angle based on maintaining a commanded airspeed. While
   * maintaining full throttle.
   * @param Va_c The commanded airspeed.
   * @param Va The current airspeed
   * @param params The parameters for the control algorithm, including the control gains.
   * @param Ts The sampling period.
   * @return The commanded pitch angle to achieve the airspeed commanded.
   */
  float airspeed_with_pitch_hold(float Va_c, float Va, const struct params_s &params, float Ts);

  /**
   * The difference in the commanded airspeed and the current airspeed.
   */
  float ap_error_;

  /**
   * The integral of the error in airspeed.
   */
  float ap_integrator_;

  /**
   * The deriviative of the error in airspeed.
   */
  float ap_differentiator_;

  /**
   * The control loop that calculates the required throttle level to move to and maintain a commanded airspeed.
   * @param Va_c The commanded airspeed.
   * @param Va The current airspeed.
   * @param params The parameters for the control algorithm, including control gains.
   * @param Ts The sampling period.
   * @return The required throttle between 0 (no throttle) and 1 (full throttle).
   */
  float airspeed_with_throttle_hold(float Va_c, float Va, const struct params_s &params, float Ts);

  /**
   * The difference between the commanded airspeed and the current airspeed.
   */
  float at_error_;

  /**
   * The integral of the error in airspeed.
   */
  float at_integrator_;

  /**
   * The derivative of the error in airspeed.
   */
  float at_differentiator_;

  /**
   * The control loop that calculates the required pitch angle to command to maintain a commanded altitude.
   * @param h_c The commanded altitude.
   * @param h The current altitude.
   * @param params The parameters for the control algorithm, including control gains.
   * @param Ts The sampling period
   * @return The commanded pitch angle to maintain and achieve the commanded altitude.
   */
  float altitiude_hold(float h_c, float h, const struct params_s &params, float Ts);

  /**
   * The difference between the commanded altitude and the current altitude.
   */
  float a_error_;

  /**
   * The integral of the error in altitude.
   */
  float a_integrator_;

  /**
   * The derivative of the error in altitude.
   */
  float a_differentiator_;

//    float cooridinated_turn_hold(float v, const struct params_s &params, float Ts); // TODO implement if you want...
//    float ct_error_;
//    float ct_integrator_;
//    float ct_differentiator_;

/**
 * Saturate a given value to a maximum or minimum of the limits.
 * @param value The value to saturate.
 * @param up_limit The maximum the value can take on.
 * @param low_limit The minimum the value can take on.
 * @return The saturated value.
 */

  float sat(float value, float up_limit, float low_limit);
};
} //end namespace

#endif // CONTROLLER_EXAMPLE_H
