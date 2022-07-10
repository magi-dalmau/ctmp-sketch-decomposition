#pragma once
#include <brfs.hpp>
#include <set>

class IWk : public BrFS {
public:
  IWk(Problem *const problem, const std::size_t k_max = 2)
      : BrFS(problem), k_max_(k_max), novelty_hash_tables_(k_max_){

                                      };

  virtual bool Solve(Plan &plan, bool lazy = false, const State *const start = nullptr) override {
    std::cout << "Solve IWk" << std::endl;
    Clear();
    for (k_ = 1; k_ <= k_max_; ++k_) {
      std::cout<<" trying to solve the problem with k="<<k_<<std::endl;
      if (BrFS::Solve(plan, lazy, start)) {
        std::cout << "Solved with K=" << k_ << std::endl;
        return true;
      } else {
        Clear();
      }
      std::cout<<"Failed to solve with k="<<k_<<std::endl;
    }
    std::cout << "Solution not found with Kmax=" << k_ << std::endl;

    return false;
  }

protected:
  virtual void Clear() override {
    num_manage_orphans_=0;
    num_susbstitutes_=0;
    BrFS::Clear();
    temp_open_queue_ = std::queue<Node *>();
    // TODO(magi.dalmau) solve
    // open_queue_ = std::queue<Node *const>();

    for (auto temp_open : temp_open_hash_table_) {
      delete temp_open.second;
    }
    temp_open_hash_table_.clear();

    for (auto &table : novelty_hash_tables_) {
      table.clear();
    }

    for (auto pruned : pruned_hash_table_) {
      delete pruned.second;
    }
    pruned_hash_table_.clear();
  }

  virtual Node *const ExtractNode() override {
    if (temp_open_hash_table_.empty()) {
      return BrFS::ExtractNode();
    } else {
      std::cout << "IWK special extracting node" << std::endl;
      auto node_temp_open = temp_open_queue_.front();

      if (open_queue_.empty()) {
        temp_open_queue_.pop();
        auto erased = temp_open_hash_table_.erase(std::hash<Node>()(*node_temp_open));
        assert(erased);
        // assert(num_open_ == temp_open_hash_table_.size() && num_open_ == temp_open_queue_.size());
        return node_temp_open;
      } else {
        auto node = open_queue_.front();
        if (node_temp_open->GetAccumulatedCost() < node->GetAccumulatedCost()) {
          temp_open_queue_.pop();
          auto erased = temp_open_hash_table_.erase(std::hash<Node>()(*node_temp_open));
          assert(erased);
          // assert(num_open_ == temp_open_hash_table_.size() && num_open_ == temp_open_queue_.size());
          return node_temp_open;
        } else {
          open_queue_.pop();
          auto erased = open_hash_table_.erase(std::hash<Node>()(*node));
          assert(erased);
          assert(num_open_ == open_hash_table_.size() && num_open_ == open_queue_.size());

          // std::cout << "Extracted " << *node->GetState() << std::endl;

          return node;
        }
      }
    }
  };

  virtual Node *const FindNodeInOpen(Node *const node) override {
    auto res = temp_open_hash_table_.find(node->GetState()->GetHash());
    if (res == temp_open_hash_table_.end()) {
      return BrFS::FindNodeInOpen(node);
    }
    return res->second;
  };

  virtual void AddToOpen(Node *const node) override {
    if (!open_queue_.empty() && node->GetAccumulatedCost() < open_queue_.front()->GetAccumulatedCost()) {
      AddToTempOpen(node);
    } else {
      BrFS::AddToOpen(node);
    }
  }

  virtual void AddToTempOpen(Node *const node) {
    std::cout << "AddToTempOpen IWk" << std::endl;

    temp_open_queue_.push(node);
    auto inserted = temp_open_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node)).second;
    assert(inserted);
    // assert(num_open_ == open_hash_table_.size() && num_open_ == open_queue_.size());
  }

  // METHODS
  std::vector<std::vector<std::size_t>> combinations(std::vector<std::size_t>::const_iterator first,
                                                     std::vector<std::size_t>::const_iterator last, std::size_t n) {
    std::vector<std::vector<std::size_t>> result;
    std::size_t l = std::distance(first, last);
    if (n == 1) {
      for (auto it = first; it != last; ++it) {
        std::vector<std::size_t> temp;
        temp.push_back(*it);
        result.push_back(temp);
      }
    } else if (n > 1 && l >= n) {
      for (auto it = first; (it + n - 1) != last; ++it) {
        for (const auto &comb : combinations(it + 1, last, n - 1)) {
          std::vector<std::size_t> temp;
          temp.push_back(*it);
          temp.insert(temp.end(), comb.begin(), comb.end());
          result.push_back(temp);
        }
      }
    }
    return result;
  }

  virtual bool Prune(Node *const node) override {
    // std::cout << "Prune IWk" << std::endl;

    auto atoms = node->GetState()->GetFeatures();
    // for_each(atoms.begin(), atoms.end(), [](const std::size_t &a) { std::cout << a << " "; });
    // std::cout << std::endl;
    std::sort(atoms.begin(), atoms.end());

    bool updated = false;
    // std::cout << k_ << " " << atoms.size() << " " << novelty_hash_tables_.size() << std::endl;
    for (std::size_t i = 1; i <= std::min(k_, atoms.size()); ++i) {
      const auto old_size = novelty_hash_tables_.at(i - 1).size();
      for (const auto &tuple : combinations(atoms.begin(), atoms.end(), i)) {
        std::size_t hash = 0;
        CombineHashVector<std::size_t>(hash, tuple);
        novelty_hash_tables_.at(i - 1)[hash].push_back(node);
      }
      if (!updated)
        updated = (old_size != novelty_hash_tables_.at(i - 1).size());
    }

    return !updated;
  }

  virtual void RepairNoveltyTable(Node *const node, std::vector<std::set<size_t>> &affected_hashes) {
    std::cout << "RepairNoveltyTable IWk" << std::endl;

    assert(affected_hashes.size() == k_);

    auto atoms = node->GetState()->GetFeatures();
    std::sort(atoms.begin(), atoms.end());

    for (std::size_t i = 1; i <= std::min(k_, atoms.size()); ++i) {

      for (const auto &tuple : combinations(atoms.begin(), atoms.end(), i)) {
        std::size_t hash = 0;
        CombineHashVector<std::size_t>(hash, tuple);
        auto iter_tuple = novelty_hash_tables_.at(i - 1).find(hash);
        if (iter_tuple == novelty_hash_tables_.at(i - 1).end()) {
          continue; // TODO: Caldria llençar algun error? se suposa que hi hauria de ser...
        }
        auto iter_node = std::find(iter_tuple->second.begin(), iter_tuple->second.end(), node);
        if (iter_node != iter_tuple->second.end()) {
          iter_tuple->second.erase(iter_node);
          if (iter_tuple->second.empty()) {
            novelty_hash_tables_.at(i - 1).erase(iter_tuple);
          } else {
            affected_hashes.at(i - 1).insert(hash);
          }
        }
      }
    }

    for (const auto successor : node->GetSuccessors()) {
      if (!isfinite(successor->GetAccumulatedCost())) {
        RepairNoveltyTable(successor, affected_hashes);
      }
    }
  }

  virtual void ManageOrphan(Node *const node) override {
    num_manage_orphans_++;
    std::cout << "ManageOrphan IWk" << std::endl;

    std::vector<std::set<size_t>> affected_hashes;
    affected_hashes.resize(k_);
    RepairNoveltyTable(node, affected_hashes);
    std::cout << "Reparation finished" << std::endl;
    for (size_t i = 1; i <= k_; i++) {
      std::cout << "The number of affected hashes is: " << affected_hashes.at(i - 1).size() << std::endl;
      for (const auto hash : affected_hashes.at(i - 1)) {
        std::cout << "Hash: " << hash << std::endl;
        auto iter = novelty_hash_tables_.at(i - 1).find(hash);
        if (iter != novelty_hash_tables_.at(i - 1).end() && !iter->second.empty()) {
          auto substitute = iter->second.at(0);
          if (FindNodeInClose(substitute)) {
            std::cout << "orphan substitute already in close" << std::endl;
            continue;
          }
          if (FindNodeInOpen(substitute)) {
            std::cout << "orphan substitute already in open" << std::endl;

            continue;
          }
          auto duplicate_pruned_iter = pruned_hash_table_.find(node->GetState()->GetHash());
          if (duplicate_pruned_iter != pruned_hash_table_.end()) {
            num_susbstitutes_++;
            AddToTempOpen(duplicate_pruned_iter->second);
            pruned_hash_table_.erase(duplicate_pruned_iter);
          } else {
            std::cout << "ERROR: Substitute node " << substitute->GetState()->GetHash() << " not found in any container"
                      << std::endl;
          }
        } else {
          continue;
        }
      }
    }
  }

  virtual Node *const FindNodeInPruned(Node *const node) override {
    auto res = pruned_hash_table_.find(node->GetState()->GetHash());
    if (res == pruned_hash_table_.end()) {
      return nullptr;
    }
    return res->second;
  };

  virtual void ManagePruned(Node *const node) override {
    std::cout << "ManagePruned IWk" << std::endl;

    auto inserted = pruned_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node)).second;
    assert(inserted);
    assert(pruned_hash_table_.size() == num_pruned_);
  }
  virtual void printStatistics() const override{
    std::cout<<"IKW statistics: "<<"managed orphans: "<<num_manage_orphans_<<" num substitutes: "<<num_susbstitutes_<< " current k is "<<k_<<std::endl;
    Search::printStatistics();
  }

      template <class T>
      inline void CombineHashVector(std::size_t &s, const std::vector<T> &vector) {
    for (const auto v : vector) {
      hash_combine<T>(s, v);
    }
  };

  template <class T> inline void hash_combine(std::size_t &s, const T &v) {
    std::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
  };
  // OBJECTS

  // SUBCLASSES

  // class NoveltyData {
  // public:
  //   void add(Node *const node) {}
  // protected:

  // };
  std::size_t k_; // current k
  std::size_t k_max_;
  std::vector<std::unordered_map<std::size_t, std::vector<Node *>>> novelty_hash_tables_;
  std::unordered_map<std::size_t, Node *const> pruned_hash_table_;
  std::queue<Node *> temp_open_queue_;
  std::unordered_map<std::size_t, Node *const> temp_open_hash_table_;
  std::size_t num_manage_orphans_, num_susbstitutes_;
};
