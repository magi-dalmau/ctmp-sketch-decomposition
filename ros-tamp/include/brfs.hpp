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
    open_queue_.push(node);
    open_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node));
  }

  virtual Node *const ExtractNode() {
    // Select Node, delete it from Open
    auto node = open_queue_.front();
    open_queue_.pop();
    open_hash_table_.erase(std::hash<Node>()(*node));

    std::cout << "Extracted " << *node->GetState() << std::endl;

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
    close_hash_table_.insert(std::make_pair(node->GetState()->GetHash(), node));
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