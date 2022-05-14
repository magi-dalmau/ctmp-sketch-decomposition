#pragma once;
#include <action.hpp>
#include <algorithm>
#include <state.hpp>
#include <vector>

class Plan {
public:
  virtual void reverse() {
    std::reverse(states.begin(), states.end());
    std::reverse(actions.begin(), actions.end());
    std::reverse(costs.begin(), costs.end());
  }
  virtual void clear(){
      states.clear();
      actions.clear();
      costs.clear();
      total_cost=0.0;
  }
  std::vector<State const *const> states;
  std::vector<Action const *const> actions;
  std::vector<double> costs;
  double total_cost;
};