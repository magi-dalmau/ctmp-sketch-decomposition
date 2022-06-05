#pragma once
#include <list>
#include <queue>
#include <search.hpp>
#include <unordered_map>

class BrFS : public Search {
public:
  BrFS(Problem *const problem) : Search(problem){};

protected:
  virtual void Clear() {
    Search::Clear();
    open_queue_ = std::queue<Node *>();
    //TODO(magi.dalmau) solve
    //open_queue_ = std::queue<Node *const>();
    
    for (auto open : open_hash_table_) {
      delete open.second;
    }
    open_hash_table_.clear();
    
    for (auto close : close_hash_table_) {
      delete close.second;
    }
    close_hash_table_.clear();
  }

  virtual void AddToOpen(Node *const node) {
    // double dist, min_dist = std::numeric_limits<double>::infinity();
    // State * min_state = nullptr;
    // for (const auto &other : open_hash_table_) {
    //   dist = other.second->GetState()->distance(node->GetState());
    //   if (dist < min_dist){
    //     min_dist = dist;
    //     delete min_state;
    //     min_state = other.second->GetState()->Clone();
    //   }
    // }
    // if (min_dist < 1e-6) { 
    //   std::cout << "Min dist: " << min_dist << " between hashes " << node->GetState()->GetHash() // << " and " << min_state->GetHash() << std::endl;
    //   std::cout << *node->GetState() << std::endl;
    //   std::cout << *min_state << std::endl;
    //   std::string x;
    //   std::cin >> x;
    // }
    // if (!std::isfinite(min_dist)) {
    //   std::cout << "Min dist is not finite: " << min_dist << std::endl;
    //   std::cout << open_hash_table_.size() << std::endl;
    //   std::string x;
    //   std::cin >> x;
    // }
    // delete min_state;

    open_queue_.push(node);
    auto inserted = open_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node)).second;
    assert(inserted);
    assert(num_open_ == open_hash_table_.size() && num_open_ == open_queue_.size());
  }

  virtual Node *const ExtractNode() {
    // Select Node, delete it from Open
    if (open_queue_.empty()) return nullptr;

    auto node = open_queue_.front();
    open_queue_.pop();
    auto erased = open_hash_table_.erase(std::hash<Node>()(*node));
    assert(erased);
    assert(num_open_ == open_hash_table_.size() && num_open_ == open_queue_.size());

    //std::cout << "Extracted " << *node->GetState() << std::endl;

    return node;
  };

  virtual Node *const FindNodeInOpen(Node *const node) {
    auto res = open_hash_table_.find(node->GetState()->GetHash());
    if (res == open_hash_table_.end()) {
      return nullptr;
    }
    return res->second;
  };

  virtual void AddToClose(Node *const node) {
    auto inserted = close_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node)).second;
    assert(inserted);
    assert(num_closed_ ==  close_hash_table_.size());
  }

  virtual Node *const FindNodeInClose(Node *const node) {
    auto res = close_hash_table_.find(node->GetState()->GetHash());
    if (res == close_hash_table_.end()) {
      return nullptr;
    }
    return res->second;
  };

  // To manage nodes to Open, 2 containers are used
  //std::queue<Node *const> open_queue_; // deque as underlying container (default option)
  std::queue<Node *> open_queue_;
  // TODO(magi.dalmau) Solve problem que of const pointers
  std::unordered_map<std::size_t, Node *const> open_hash_table_;

  std::unordered_map<std::size_t, Node *const> close_hash_table_;
};