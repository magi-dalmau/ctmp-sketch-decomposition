#pragma once
#include <action.hpp>
#include <plan.hpp>
#include <state.hpp>
#include <vector>

class Problem {
public:
  virtual ~Problem() {
    for (auto action : actions_)
      delete action;
  }
  virtual State *const Start() const = 0;
  virtual bool IsGoal(State const *const state) = 0;
  virtual std::vector<Action *> GetValidActions(State const *const state, bool lazy = false) {
    std::vector<Action *> valid_actions;
    for (const auto action : actions_) {
      if (IsActionValid(state, action, lazy)) {
        valid_actions.push_back(action->Clone());
      }
    }
    return valid_actions;
  };
  virtual bool SetActiveSketchRule(const State *const state) { return true; }
  virtual void AdaptativeSampling(const State *const state) {}

  virtual bool IsActionValid(State const *const state, Action *const action, bool lazy = false) { return true; };
  virtual State *const GetSuccessor(State const *const state, Action const *const action) = 0;
  virtual double GetCost(State const *const state, Action const *const action) { return 1.; };
  virtual void print(std::ostream &os) const = 0;
  // Overloading << operator
  friend std::ostream &operator<<(std::ostream &os, const Problem &obj) {
    obj.print(os);
    return os;
  }
  virtual void PrintStatistics() const = 0;
  virtual bool ExecutePlan(const Plan &plan) { return false; }
  virtual void UpdateGoalPositions(State *const state) {}

protected:
  std::vector<Action *> actions_;
};