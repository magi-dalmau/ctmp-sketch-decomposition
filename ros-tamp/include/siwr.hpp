#pragma once
#include <iwk.hpp>

class SIWr : public IWk {
public:
  SIWr(Problem *const problem, const std::size_t k_max = 2)
      : IWk(problem, k_max){

        };

  virtual bool Solve(Plan &plan, bool lazy = false, const State *const start = nullptr) override {
    std::cout << "Solve SIWr" << std::endl;
    ClearIWR();
    Clear();
    const State *start_state = nullptr;
    const State *prev_start_state = nullptr;

    start_state = start ? start->Clone() : problem_->Start();
    bool failed = false;
    bool succeeded = false;

    while (!failed && !succeeded) {
      Plan sub_plan;
      printStatisticsSIWR();
      problem_->AdaptativeSampling(start_state);
      problem_->SetActiveSketchRule(start_state);
      failed = !(IWk::Solve(sub_plan, lazy, start_state));
      UpdateSIWRStatistics();
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
    printStatisticsSIWR();
    if (!succeeded)
      plan.clear();
    return succeeded;
  }

  virtual void ClearIWR() {
    num_subproblems_solved_ = 0;
    siwr_num_open_ = 0;
    siwr_num_duplicates_open_ = 0;
    siwr_num_closed_ = 0;
    siwr_num_duplicates_closed_ = 0;
    siwr_num_pruned_ = 0;
    siwr_num_processed_ = 0;
    siwr_num_rewired_ = 0;
    siwr_num_plans_ = 0;
    siwr_num_already_confirmed_ = 0;
    siwr_num_duplicates_pruned_ = 0;
  }

  virtual void printStatisticsSIWR() const {
    std::cout << "Subproblems solved: " << num_subproblems_solved_ << "\n SIWr global statistics: " << std::endl;
    std::cout << "Total Open: " << siwr_num_open_ << " Total Dup-Open: " << siwr_num_duplicates_open_
              << " Total Closed: " << siwr_num_closed_ << " Total Dup-Closed: " << siwr_num_duplicates_closed_
              << " Total Pruned: " << siwr_num_pruned_ << " Total Dup-Pruned: " << siwr_num_duplicates_pruned_
              << " Total Processed: " << siwr_num_processed_ << " Total Rewired: " << siwr_num_rewired_
              << " Total Get Plan: " << siwr_num_plans_ << " Total Already confirmed: " << siwr_num_already_confirmed_
              << std::endl;
  }

protected:
  void UpdateSIWRStatistics() {
    siwr_num_open_ += num_open_;
    siwr_num_duplicates_open_ += num_duplicates_open_;
    siwr_num_closed_ += num_closed_;
    siwr_num_duplicates_closed_ += num_duplicates_closed_;
    siwr_num_pruned_ += num_pruned_;
    siwr_num_processed_ += num_processed_;
    siwr_num_rewired_ += num_rewired_;
    siwr_num_plans_ += num_plans_;
    siwr_num_already_confirmed_ += num_already_confirmed_;
    siwr_num_duplicates_pruned_ += num_duplicates_pruned_;
  }

  // Extra statistics:
  std::size_t num_subproblems_solved_;

  std::size_t siwr_num_open_, siwr_num_duplicates_open_, siwr_num_closed_, siwr_num_duplicates_closed_,
      siwr_num_pruned_, siwr_num_processed_, siwr_num_rewired_, siwr_num_plans_, siwr_num_already_confirmed_,
      siwr_num_duplicates_pruned_;
};