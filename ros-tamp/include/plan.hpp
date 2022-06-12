#pragma once
#include <action.hpp>
#include <algorithm>
#include <iostream>
#include <state.hpp>
#include <vector>

class Plan {
public:
  ~Plan() { clear(); }

  virtual void reverse() {
    std::reverse(states.begin(), states.end());
    std::reverse(actions.begin(), actions.end());
    std::reverse(costs.begin(), costs.end());
  }

  void copy(Plan &plan) const {
    plan.clear();

    plan.states.push_back(states.at(0)->Clone());
    for (std::size_t i = 0; i < actions.size(); ++i) {
      plan.states.push_back(states.at(i + 1)->Clone());
      plan.actions.push_back(actions.at(i)->Clone());
      plan.costs.push_back(costs.at(i));
    }
    plan.total_cost = total_cost;
  }

  virtual void append(const Plan &other) {
    for (std::size_t i = 0; i < other.actions.size(); ++i) {
      states.push_back(other.states.at(i + 1)->Clone());
      actions.push_back(other.actions.at(i)->Clone());
      costs.push_back(other.costs.at(i));
    }
    total_cost += other.total_cost;
  }

  virtual void clear() {
    for (auto state : states)
      delete state;
    states.clear();

    for (auto action : actions)
      delete action;
    actions.clear();

    costs.clear();

    total_cost = 0.0;
  }

  virtual void print(std::ostream &os) const {
    std::cout << "Plan found from root " << *states.front() << " to goal " << *states.back() << "\nTotal cost "
              << total_cost << std::endl;
    std::cout << "Path:" << std::endl;
    std::cout << *states.front() << std::endl;
    for (std::size_t k = 0; k < actions.size(); ++k) {
      std::cout << "\t" << *actions.at(k) << " with cost " << costs.at(k) << std::endl;
      std::cout << *states.at(k + 1) << std::endl;
    }
  };
  // Overloading << operator
  friend std::ostream &operator<<(std::ostream &os, const Plan &obj) {
    obj.print(os);
    return os;
  }

  // std::vector<State const * const> states;
  // std::vector<Action const * const> actions;
  // TODO(magi.dalmau) Solve problem vector of const pointers
  std::vector<State const *> states;
  std::vector<Action const *> actions;
  std::vector<double> costs;
  double total_cost;
};