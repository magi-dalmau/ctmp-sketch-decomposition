#pragma once;
#include <list>
#include <queue>
#include <search.hpp>
#include <unordered_set>

class BrFS : Search {
public:
  BrFS(Problem *const problem) : Search(problem){};

protected:
  virtual void AddToOpen(Node *const node) { open_.push(node); }

  virtual Node *const ExtractNode() {
    // Select Node, delete node from Open
    auto node = open_.front();
    open_.pop();
    return node;
  };
  

  virtual void AddToClose(Node *const node) { close_.insert(node); }

  std::queue<Node *const, std::list<Node *const>> open_;
  std::unordered_set<Node *const> close_;
};