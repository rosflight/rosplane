#ifndef ESTIMATOR_INTERFACE_HPP
#define ESTIMATOR_INTERFACE_HPP

namespace rosplane
{
class EstimatorInterface
{
public:
  virtual ~EstimatorInterface() = default;

  struct Input
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float static_pres;
    float diff_pres;
    bool gps_new;
    float gps_n;
    float gps_e;
    float gps_h;
    float gps_Vg;
    float gps_course;
    bool status_armed;
    bool armed_init;
  };

  struct Output
  {
    float pn;
    float pe;
    float h;
    float va;
    float alpha;
    float beta;
    float phi;
    float theta;
    float psi;
    float chi;
    float p;
    float q;
    float r;
    float Vg;
    float wn;
    float we;
  };

  virtual void estimate(const Input & input, Output & output) = 0;
};
} // namespace rosplane

#endif //ESTIMATOR_INTERFACE_HPP
