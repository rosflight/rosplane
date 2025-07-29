#ifndef CONTROLLER_EXAMPLE_H
#define CONTROLLER_EXAMPLE_H

#include "controller_state_machine.hpp"

namespace rosplane
{

class ControllerSucessiveLoop : public ControllerStateMachine
{
public:
  /**
   * Constructor to initialize node.
   */
  ControllerSucessiveLoop();

protected:
  /**
   * This function continually loops while the aircraft is in the take-off zone. The lateral and longitudinal control
   * for the take-off zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the climb zone. The lateral and longitudinal control
   * for the climb zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the altitude hold zone. The lateral and longitudinal 
   * control for the altitude hold zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void altitude_hold(const Input & input, Output & output);

  /**
   * This function runs when the aircraft exits the take-off zone. Any changes to the controller that need to happen
   * only once as the aircraft exits take-off mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void take_off_exit();

  /**
   * This function runs when the aircraft exits the climb zone. Any changes to the controller that need to happen
   * only once as the aircraft exits climb mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void climb_exit();

  /**
   * This function runs when the aircraft exits the altitude hold zone (usually a crash). Any changes to the controller that 
   * need to happen only once as the aircraft exits altitude mode should be placed here. This sets differentiators and
   * integrators to 0.
   */
  virtual void altitude_hold_exit();

  /**
   * This function runs the lateral control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_longitudinal_control(const Input & input, Output & output);

  /**
   * The control loop for moving to and holding a commanded course.
   * @param chi_c The commanded course angle.
   * @param chi The current course angle.
   * @param phi_ff The roll angle feedforward term. This allows for faster convergence.   // TODO add reference to book?
   * @param r The yaw rate taken from the gyro.
   * @return The commanded roll angle, to achieve the course angle.
   */
  float course_hold(float chi_c, float chi, float phi_ff, float r);

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
   * @return The aileron deflection in radians required to achieve the commanded roll angle.
   */
  float roll_hold(float phi_c, float phi, float p);

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
   * @return The elevator deflection in radians required to achieve the commanded pitch.
   */
  float pitch_hold(float theta_c, float theta, float q);

  /**
   * The difference between the commanded pitch angle and the current pitch angle.
   */
  float p_error_;

  /**
   * The integral of the error in pitch angle.
   */
  float p_integrator_;

  /**
   * The control loop that calculates the required throttle level to move to and maintain a commanded airspeed.
   * @param va_c The commanded airspeed.
   * @param va The current airspeed.
   * @return The required throttle between 0 (no throttle) and 1 (full throttle).
   */
  float airspeed_with_throttle_hold(float va_c, float va);

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
   * @return The commanded pitch angle to maintain and achieve the commanded altitude.
   */
  float altitude_hold_control(float h_c, float h);

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
  float yaw_damper(float r);

  float delta_r_delay_;
  float r_delay_;

  /**
 * Saturate a given value to a maximum or minimum of the limits.
 * @param value The value to saturate.
 * @param up_limit The maximum the value can take on.
 * @param low_limit The minimum the value can take on.
 * @return The saturated value.
 */

  float sat(float value, float up_limit, float low_limit);

  float adjust_h_c(float h_c, float h, float max_diff);

private:
  /**
   * Declares the parameters associated to this controller, controller_successive_loop, so that ROS2 can see them.
   * Also declares default values before they are set to the values set in the launch script.
  */
  void declare_parameters();
};
} // namespace rosplane

#endif // CONTROLLER_EXAMPLE_H
