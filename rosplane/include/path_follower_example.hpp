#ifndef PATH_FOLLOWER_EXAMPLE_H
#define PATH_FOLLOWER_EXAMPLE_H

#include "path_follower_base.hpp"

namespace rosplane
{

class path_follower_example : public path_follower_base
{
public:
  path_follower_example();

private:
  virtual void follow(const struct input_s & input,
                      struct output_s & output);
};

} // namespace rosplane
#endif // PATH_FOLLOWER_EXAMPLE_H
