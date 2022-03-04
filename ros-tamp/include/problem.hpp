#pragma once
#include <action.hpp>
#include <state.hpp>
#include <vector>

class Problem {
public:
  virtual State *const start() = 0;
  virtual bool IsGoal(State const *const state) const = 0;
  virtual std::vector<Action *> GetValidActions(State const *const state, bool lazy = false) {
    std::vector<Action *> valid_actions;
    for (const auto action : actions_) {
      if (IsActionValid(state, action, lazy)) {
        valid_actions.push_back(action);
      }
    }
    return valid_actions;
  };

  virtual bool IsActionValid(State const *const state, Action const *const action, bool lazy = false) = 0;
  virtual State *const GetSuccessor(State const *const state, Action const *const action) = 0;
  virtual double GetCost(State const *const state, Action const *const action) { return 1.; };

protected:
  std::vector<Action *> actions_;
};
