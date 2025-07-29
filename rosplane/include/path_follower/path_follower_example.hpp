#ifndef PATH_FOLLOWER_EXAMPLE_H
#define PATH_FOLLOWER_EXAMPLE_H

#include "path_follower_base.hpp"

namespace rosplane
{

class PathFollowerExample : public PathFollowerBase
{
public:
  PathFollowerExample();

private:
  virtual void follow(const Input & input, Output & output);
};

} // namespace rosplane
#endif // PATH_FOLLOWER_EXAMPLE_H
