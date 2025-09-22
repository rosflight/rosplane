#include "ekf/estimator_continuous_discrete.hpp"

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane::EstimatorContinuousDiscrete>());

  return 0;
}
