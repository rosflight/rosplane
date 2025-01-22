#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include <Eigen/Eigen>

#include "path_manager_base.hpp"

#define M_PI_F 3.14159265358979323846f
#define M_PI_2_F 1.57079632679489661923f

namespace rosplane
{

enum class FilletState
{
  STRAIGHT,
  TRANSITION,
  ORBIT
};

enum class DubinState
{
  FIRST,
  BEFORE_H1,
  BEFORE_H1_WRONG_SIDE,
  STRAIGHT,
  BEFORE_H3,
  BEFORE_H3_WRONG_SIDE
};

class PathManagerExample : public PathManagerBase
{
public:
  PathManagerExample();

private:
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  FilletState fil_state_;

  bool first_;

  /**
   * @brief Determines the line type and calculates the line parameters to publish to path_follower
   */
  virtual void manage(const Input & input, Output & output);

  /**
   * @brief Calculates the most convenient orbit direction based on the orientation of the vehicle relative to the orbit center
   * 
   * @param pn: North position of the vehicle
   * @param p3: East position of the vehicle
   * @param c_n: North position of the orbit center
   * @param c_e: East position of the orbit center
   * 
   * @return Integer value representing the orbit direction (-1 or 1)
   */
  int orbit_direction(float pn, float pe, float chi, float c_n, float c_e);

  /**
   * @brief Increments the indices of the waypoints currently used in the "manage" calculations
   * 
   * @param idx_a: Index of the most recently achieved point
   * @param idx_b: Index of the next target waypoint
   * @param idx_c: Index of the waypoint after the next target waypoint
   * @param input: Struct containing the state of the vehicle
   * @param output: Struct that will contain all of the information about the desired line to pass to the path follower
   */
  void increment_indices(int & idx_a, int & idx_b, int & idx_c, const Input & input,
                         Output & output);

  /**
   * @brief Manages a straight line segment. Calculates the appropriate line parameters to send to the path follower
   * 
   * @param input: Input struct that contains some of the state of the vehicle
   * @param output: Output struct containing the information about the desired line
   */
  void manage_line(const Input & input, Output & output);

  /**
   * @brief Manages a fillet line segment. Calculates the appropriate line parameters to send to the path follower
   * 
   * @param input: Input struct that contains some of the state of the vehicle
   * @param output: Output struct containing the information about the desired line
   */
  void manage_fillet(const Input & input, Output & output);

  /**
   * @brief Manages a Dubins path segment. Calculates the appropriate line parameters to send to the path follower
   * 
   * @param input: Input struct that contains some of the state of the vehicle
   * @param output: Output struct containing the information about the desired line
   */
  void manage_dubins(const Input & input, Output & output);

  DubinState dub_state_;

  struct DubinsPath
  {

    Eigen::Vector3f ps; /** the start position */
    float chis;         /** the start course angle */
    Eigen::Vector3f pe; /** the end position */
    float chie;         /** the end course angle */
    float R;            /** turn radius */
    float L;            /** length of the path */
    Eigen::Vector3f cs; /** center of the start circle */
    int lams;           /** direction of the start circle */
    Eigen::Vector3f ce; /** center of the endcircle */
    int lame;           /** direction of the end circle */
    Eigen::Vector3f w1; /** vector defining half plane H1 */
    Eigen::Vector3f q1; /** unit vector along striaght line path */
    Eigen::Vector3f w2; /** vector defining half plane H2 */
    Eigen::Vector3f w3; /** vector defining half plane H3 */
    Eigen::Vector3f q3; /** unit vector defining direction of half plane H3 */
  };
  DubinsPath dubins_path_;

  /**
   * @brief Calculates the parameters of a Dubins path
   * 
   * @param start_node: Starting waypoint of the Dubins path
   * @param end_node: Ending waypoint of the Dubins path
   * @param R: Minimum turning radius R
   */
  void dubins_parameters(const Waypoint start_node, const Waypoint end_node, float R);

  /**
   * @brief Computes the rotation matrix for a rotation in the z plane (normal to the Dubins plane)
   * 
   * @param theta: Rotation angle
   * 
   * @return 3x3 rotation matrix
   */
  Eigen::Matrix3f rotz(float theta);

  /**
   * @brief Wraps an angle to 2*PI
   * 
   * @param in: Angle to wrap
   * 
   * @return Angle wrapped to within 2*PI
   */
  float mo(float in);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a parameter file
   */
  void declare_parameters();
};
} // namespace rosplane
#endif // PATH_MANAGER_EXAMPLE_H
