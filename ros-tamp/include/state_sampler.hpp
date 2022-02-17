#include <state_space.hpp>

class StateSampler {
public:
  virtual void sampleUniform(State *state) = 0;
};