#pragma once

#include <map>
#include <node.hpp>
#include <plan.hpp>
#include <problem.hpp>
#include <state.hpp>
// template < typename NodeContainer>
class Search {
public:
  Search(Problem *const problem) : problem_(problem){};
  virtual ~Search() = default;
  // TODO return plan
  bool solve(Plan &plan) {
    root_node_ = new Node(problem_->start());
    AddToOpen(root_node_);
    auto node = ExtractNode();

    if (problem_->IsGoal(node->GetState()))
      return true;

    while (node) {
      auto end = Process(node, plan);
      if (end)
        return true;
      node = ExtractNode();
    }

    return false;
  }

protected:
  virtual Node *const Process(Node *const parent, Plan &plan) {

    for (const auto action : problem_->GetValidActions(parent->GetState())) {
      double action_cost = problem_->GetCost(parent->GetState(), action);
      auto successor = new Node(problem_->GetSuccessor(parent->GetState(), action), parent, action, action_cost);

      // TODO(magi.dalmau) Managafe duplicats and is goal in order to fit the new lazy approach (orphan nodes connected
      // to goal)
      if (Prune(successor)) {
        delete successor;
        continue;
      }
      auto duplicate_open = FindNodeInOpen(successor);

      if (duplicate_open) {
        duplicate_open->AddParent(parent, action, action_cost);
        ManageDuplicateInOpen(duplicate_open);
        parent->AddSuccessor(duplicate_open);
        delete successor;
        continue;
      }

      auto duplicate_close = FindNodeInClose(successor);

      if (duplicate_close) {
        duplicate_close->AddParent(parent, action, action_cost);
        ManageDuplicateInOpen(duplicate_close);
        parent->AddSuccessor(duplicate_close);
        auto connected_goal = duplicate_close->GetConnectedGoal();
        if (connected_goal && (GetPlan(plan, connected_goal))) {
          return connected_goal;
        }
        delete successor;
        continue;
      }

      AddToOpen(successor);
      parent->AddSuccessor(successor);

      if (problem_->IsGoal(successor->GetState()) && (GetPlan(plan, successor))) {
        return successor;
      }
    }
    return NULL;
  };

  virtual bool Prune(Node *const node) const { return false; }

  virtual bool InOpen(Node *const node) const = 0;

  virtual bool IsClosed(Node *const node) const = 0;

  virtual bool AddToOpen(Node *const node) = 0;

  virtual Node *const FindNodeInOpen(Node *const node) = 0;

  virtual void ManageDuplicateInOpen(Node *const node) = 0;

  virtual Node *const FindNodeInClose(Node *const node) = 0;

  virtual void ManageDuplicateInClose(Node *const node) = 0;

  virtual Node *const ExtractNode() = 0; // Select Node, delete node from Open

  virtual bool GetPlan(Plan &plan, Node *const goal) {
    plan.clear();
    Node *current_node = goal;
    plan.states.push_back(current_node->GetState());
    plan.total_cost =
        current_node->GetAccumulatedCost(); // TODO(magi.dalmau) think about if its necesary save individual action
                                            // costs instead of only accumulated cost from roo to a certain node
    while (current_node != root_node_) {

      auto action = current_node->GetAction();
      auto parent = current_node->GetParent();
      if (problem_->IsActionValid(parent->GetState(), action, false)) {
        current_node = parent;
        current_node->SetConnectedGoal(goal);
        plan.states.push_back(current_node->GetState());
        plan.actions.push_back(action);
      } else {
        current_node->RemoveParent();

        return GetPlan(plan, goal);
      }
    }

    plan.reverse();
    return true;
  };

  // vars
  Problem *const problem_;
  Node *root_node_;
  // std::size_t closed_nodes;
};