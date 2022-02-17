#pragma once

// STD includes
#include <vector>
#include <string>
#include <map>

// ros-tamp includes
#include <agent.hpp>
#include <goal.hpp>
#include <state.hpp>

class Problem {
public:
  State *start;
  Goal *goal;
};

class Domain {
public:
  virtual State getSuccessor(const Action &action, const State &state) = 0;
  virtual double actionCost(const Action &action) { return 1; };
  std::map<std::string, std::vector<Action>> getValidActions(const State *state);

protected:
  std::vector<Agent> agents;
};