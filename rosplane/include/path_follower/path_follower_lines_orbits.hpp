#ifndef PATH_FOLLOWER_LINES_ORBITS_H
#define PATH_FOLLOWER_LINES_ORBITS_H

#include "path_follower_ros.hpp"

namespace rosplane
{

class PathFollowerLinesOrbits : public PathFollowerROS
{
public:
  PathFollowerLinesOrbits();

private:
  virtual void follow(const Input & input, Output & output);
};

} // namespace rosplane
#endif // PATH_FOLLOWER_LINES_ORBITS_H
