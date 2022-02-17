#pragma once

//ros-tamp includes
#include <state.hpp>

class Goal {
public:
  virtual bool isGoal(const State &state) = 0;
};