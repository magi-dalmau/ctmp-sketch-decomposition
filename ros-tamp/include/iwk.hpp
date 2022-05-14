#pragma once
#include <brfs.hpp>

class IWk : public BrFS {
public:
  IWk(Problem *const problem, const std::size_t k_max = 2) : BrFS(problem), k_max_(k_max){};

  virtual bool solve(Plan &plan, bool lazy = false) override {
    k_ = 0;
    while (k_ < k_max_) {
      smallest_exceeded_novelty_ = k_max_;
      if (BrFS::solve(plan, lazy)) {
        return true;
      }
      if (smallest_exceeded_novelty_ <= k_) {
        throw std::runtime_error("k already tested ");
      }
      k_ = smallest_exceeded_novelty_;
    }
  }

protected:
  virtual bool Prune(Node *const node) {
    std::size_t novelty=problem_->GetNovelty(node->GetState());    
    if (novelty > k_) {
      if (novelty < smallest_exceeded_novelty_) {
        smallest_exceeded_novelty_ = novelty;
      }
      return true;
    } else {
      return false;
    }
  }

  std::size_t k_max_;
  std::size_t smallest_exceeded_novelty_;
  std::size_t k_; // current k
};
