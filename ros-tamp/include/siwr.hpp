#pragma once
#include <iwk.hpp>

class SIWr : public IWk {
public:
  SIWr(Problem *const problem, const std::size_t k_max = 2)
      : IWk(problem, k_max){

        };

  virtual bool Solve(Plan &plan, bool lazy = false, State *const start = nullptr) override {
    std::cout << "Solve SIWr" << std::endl;
    Clear();
    State *start_state = nullptr;
    State *prev_start_state = nullptr;

    start_state = start ? start->Clone() : problem_->Start();
    bool failed = false;
    bool succeeded = false;
    
    while (!failed && !succeeded) {
      problem_->SetActiveSketchRule(start);
      failed = !(IWk::Solve(plan, lazy, start));
      if (!failed) {
        if (prev_start_state && start_state->GetHash() == prev_start_state->GetHash()) {
          succeeded = true;
        } else {
          delete prev_start_state; // TODO: cal?
          prev_start_state = start_state;
          delete start_state; // TODO: cal?
          start_state = plan.states.back()->Clone();
        }
      }
    }
    delete start_state;
    delete prev_start_state;
    if (succeeded) {
      return true;
    } else {
      return false;
    }
  }

protected:
};