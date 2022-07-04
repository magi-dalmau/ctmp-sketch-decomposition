#pragma once
#include <action.hpp>
#include <limits>
#include <queue>
#include <state.hpp>

class Node {
public:
  // CONSTRUCTORS
  Node(State const *const state, bool is_root = true)
      : state_(state) { // Generetate node without parent. Example use-case search root node
    is_root_ = is_root;
    connected_goal_ = nullptr;
  };

  Node(State const *const state, Node *const parent, Action const *const action, double action_cost) : state_(state) {
    is_root_ = false;
    AddParent(parent, action, action_cost);
    connected_goal_ = nullptr;
  };
  ~Node() {
    delete state_;
    while (!edges_.empty()) {
      auto edge = edges_.top();
      edges_.pop();
      delete edge.action;
    }
  }

  // FUNCTIONS
  bool AddParent(Node *const parent, Action const *const action, double action_cost) {
    if (!is_root_) {
      const double old_accumulated_cost = GetAccumulatedCost();
      // if (GetParent()) {
      //   std::cout << "Parent of " << *state_ << " with acc cost " << old_accumulated_cost << " is "
      //             << *GetParent()->GetState() << std::endl;
      // } else {
      //   std::cout << *state_ << " with acc cost " << old_accumulated_cost << " has no parent" << std::endl;
      // }

      Edge edge = {parent, action, action_cost, action_cost + parent->GetAccumulatedCost(), false};
      edges_.push(edge);

      const double new_accumulated_cost = GetAccumulatedCost();
      // std::cout << "Parent of " << *state_ << " with acc cost " << new_accumulated_cost << " is "
      //           << *GetParent()->GetState() << std::endl;
      if (old_accumulated_cost != new_accumulated_cost) {
        for (auto &successor : successors_) {
          successor->UpdateEdgeCost(this, new_accumulated_cost);
        }
        return true;
      }
    }
    return false;
  };

  void RemoveParent() {
    if (!edges_.empty()) {
      std::cout << __LINE__ << std::endl;
      const double old_accumulated_cost = GetAccumulatedCost();
      std::cout << __LINE__ << std::endl;
      edges_.pop();
      std::cout << __LINE__ << std::endl;
      const double new_accumulated_cost = GetAccumulatedCost();
      std::cout << __LINE__ << std::endl;
      if (old_accumulated_cost != new_accumulated_cost) {
        std::cout << __LINE__ << std::endl;
        for (auto &successor : successors_) {
          std::cout << __LINE__ << std::endl;
          successor->UpdateEdgeCost(this, new_accumulated_cost);
          std::cout << __LINE__ << std::endl;
        }
        std::cout << __LINE__ << std::endl;
      }
      std::cout << __LINE__ << std::endl;
    }
    // TODO(magi.dalmau) gestionar excepcio no hi havia parents
  };

  Node *const GetParent() const {
    if (!edges_.empty()) {
      return edges_.top().parent;
    } else {
      return nullptr;
    }
  };

  void ConfirmParent() {
    if (!edges_.empty()) {
      auto edge = edges_.top();
      edges_.pop();
      edge.non_lazy_validated = true;
      edges_.push(edge);
    }
  }

  bool IsParentConfirmed() const { return edges_.top().non_lazy_validated; }

  void UpdateEdgeCost(const Node *const parent, double new_parent_cost) {
    std::cout << __LINE__<< std::endl;
    const double old_accumulated_cost = GetAccumulatedCost();
    bool found = false;
    std::vector<Edge> temp_edges;

    while (!found && !edges_.empty()) {
      Edge current_edge = edges_.top();
      edges_.pop();
      if (current_edge.parent == parent) {
        current_edge.acumulated_cost = new_parent_cost + current_edge.action_cost;
        found = true;
      }
      temp_edges.push_back(current_edge);
    }
    for (const auto &edge : temp_edges) {
      edges_.push(edge);
    }
    const double new_accumulated_cost = GetAccumulatedCost();
    if (old_accumulated_cost != new_accumulated_cost) {
      for (auto &successor : successors_) {
        successor->UpdateEdgeCost(this, new_accumulated_cost);
      }
    }
    std::cout << __LINE__ << std::endl;
  }

  void AddSuccessor(Node *const successor) {
    if (!successor->is_root_) {
      successors_.push_back(successor);
    }
  };

  std::vector<Node *> GetSuccessors() { return successors_; }

  Action const *const GetAction() const {
    if (edges_.size() > 0) {
      return edges_.top().action;
    } else {
      return nullptr;
    }
  };

  double GetAccumulatedCost() const {
    if (is_root_) {
      return 0.0;
    }
    if (edges_.size() > 0) {
      return edges_.top().acumulated_cost;
    } else {
      return std::numeric_limits<double>::infinity();
    }
  };

  double GetActionCost() const {
    if (is_root_) {
      return 0.0;
    }
    if (edges_.size() > 0) {
      return edges_.top().action_cost;
    } else {
      return std::numeric_limits<double>::infinity();
    }
  };

  State const *const GetState() const { return state_; }

  // std::vector<Node *const> GetSuccessors() const { return successors_; }

  void SetConnectedGoal(Node *goal) { connected_goal_ = goal; }

  Node *GetConnectedGoal() { return connected_goal_; }

protected:
  struct Edge {
    // Node *const parent;
    // Action const *const action;
    // TODO(magi.dalmau) solve const problem
    Node *parent;
    Action const *action;
    double action_cost;
    double acumulated_cost;
    bool non_lazy_validated;
    // bool operator<(const Edge &rhs) { return acumulated_cost < rhs.acumulated_cost; }
  };
  class Compare {
  public:
    bool operator()(const Edge &lhs, const Edge &rhs) { return lhs.acumulated_cost > rhs.acumulated_cost; }
  };

  bool is_root_;
  State const *const state_;
  std::priority_queue<Edge, std::vector<Edge>, Compare> edges_;
  // std::vector<Node *const> successors_;
  // TODO(magi.dalmau) Solve vector of const pointers problem
  std::vector<Node *> successors_;
  Node *connected_goal_;
};

namespace std {
template <> struct hash<Node> {
  size_t operator()(const Node &node) const { return node.GetState()->GetHash(); }
};
} // namespace std