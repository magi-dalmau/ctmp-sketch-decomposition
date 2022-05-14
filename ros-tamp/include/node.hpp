#pragma once;
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
  ~Node() { delete state_; }

  // FUNCTIONS
  void AddParent(Node *const parent, Action const *const action, double action_cost) {

    Edge edge = {parent, action, action_cost, action_cost + parent->GetAccumulatedCost()};
    edges_.push(edge);
  };

  void RemoveParent() {
    if (!edges_.empty()) {
      const double old_accumulated_cost = GetAccumulatedCost();
      edges_.pop();
      const double new_accumulated_cost = GetAccumulatedCost();
      if (old_accumulated_cost != new_accumulated_cost) {
        for (auto &successor : successors_) {
          successor->UpdateEdgeCost(this, new_accumulated_cost);
        }
      }
    }
    // TODO(magi.dalmau) gestionar excepcio no hi havia parents
  };

  void UpdateEdgeCost(const Node *const parent, double new_parent_cost) {

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
  }

  void AddSuccessor(Node *const successor) { successors_.push_back(successor); };

  Node *const GetParent() const {
    if (edges_.size() > 0) {
      return edges_.top().parent;
    } else {
      return nullptr;
    }
  };

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

  State const *const GetState() const { return state_; }

  std::vector<Node *const> GetSuccessors() { return successors_; }

  void SetConnectedGoal(Node *goal) { connected_goal_ = goal; }

  Node *GetConnectedGoal() { return connected_goal_; }

protected:
  struct Edge {
    Node *const parent;
    Action const *const action;
    double action_cost;
    double acumulated_cost;
    bool operator<(const Edge &rhs) { return acumulated_cost < rhs.acumulated_cost; }
  };
  bool is_root_;
  State const *const state_;
  std::priority_queue<Edge, std::vector<Edge>, std::less<Edge>> edges_;
  std::vector<Node *const> successors_;
  Node *connected_goal_;
};
