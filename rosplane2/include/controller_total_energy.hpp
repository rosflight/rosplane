#ifndef BUILD_CONTROLLER_TOTAL_ENERGY_H
#define BUILD_CONTROLLER_TOTAL_ENERGY_H

#include "controller_successive_loop.hpp"

namespace rosplane2
{

    class controller_total_energy : public controller_successive_loop
{
  public:
  /**
   * Constructor to initialize node.
   */
  controller_total_energy();

  protected:

  virtual void take_off_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output);
  virtual void climb_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output);
  virtual void alt_hold_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output);

  virtual void take_off_exit();
  virtual void climb_exit();
  virtual void altitude_hold_exit();

  float total_energy_throttle(float va_c, float va, float h_c, float h, const struct params_s &params, float Ts);
  float total_energy_pitch(float va_c, float va, float h_c, float h, const struct params_s &params, float Ts);

  float E_integrator_;

  float L_integrator_;


};
} //end namespace


#endif //BUILD_CONTROLLER_TOTAL_ENERGY_H
