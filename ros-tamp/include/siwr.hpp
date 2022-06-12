#pragma once
#include <iwk.hpp>

class SIWr : public IWk {
public:
  SIWr(Problem *const problem, const std::size_t k_max = 2)
      : IWk(problem, k_max){

        };

  virtual bool Solve(Plan &plan, bool lazy = false, const State *const start = nullptr) override {
    std::cout << "Solve SIWr" << std::endl;
    Clear();
    const State *start_state = nullptr;
    const State *prev_start_state = nullptr;

    start_state = start ? start->Clone() : problem_->Start();
    bool failed = false;
    bool succeeded = false;

    while (!failed && !succeeded) {
      Plan sub_plan;
      printStatistics();
      problem_->SetActiveSketchRule(start_state);
      failed = !(IWk::Solve(sub_plan, lazy, start_state));
      if (!failed) {
        num_subproblems_solved_++;
        if (plan.states.empty()) {
          sub_plan.copy(plan);
        } else {
          plan.append(sub_plan);
        }
        if (prev_start_state && start_state->GetHash() == prev_start_state->GetHash()) {
          succeeded = true;
        } else {
          delete prev_start_state;
          prev_start_state = start_state;
          start_state = plan.states.back()->Clone();
        }
      }
    }
    delete start_state;
    delete prev_start_state;
    printStatistics();
    if (!succeeded)
      plan.clear();
    return succeeded;
  }

  virtual void Clear() override {
    IWk::Clear();
    num_subproblems_solved_ = 0;
  }

  virtual void printStatistics() const override {
    std::cout << "Subproblems solved: " << num_subproblems_solved_ << "\n Current subproblem statistics: " << std::endl;
    IWk::printStatistics();
  }

protected:
  // Extra statistics:
  std::size_t num_subproblems_solved_;
};