#pragma once

#include <assert.h>
#include <iostream>
#include <map>
#include <math.h>
#include <node.hpp>
#include <plan.hpp>
#include <problem.hpp>
#include <state.hpp>

// template < typename NodeContainer>
class Search {
public:
  Search(Problem *const problem) : problem_(problem){};
  virtual ~Search() { Clear(); };

  bool solve(Plan &plan, bool lazy = false) {
    std::cout << "Starting solver" << std::endl;
    Clear();

    root_node_ = new Node(problem_->start());
    AddToOpen(root_node_);
    auto node = ExtractNode();

    if (problem_->IsGoal(node->GetState())) {
      GetPlan(plan, node);
      return true;
    }

    while (node) {
      auto end = Process(node, plan, lazy);

      if (end)
        return true;

      node = ExtractNode();
    }

    return false;
  }

protected:
  virtual Node *const Process(Node *const parent, Plan &plan, bool lazy) {
    std::cout << "Starting processing " << *parent->GetState() << std::endl;
    for (const auto action : problem_->GetValidActions(parent->GetState(), lazy)) {
      double action_cost = problem_->GetCost(parent->GetState(), action);
      auto successor = new Node(problem_->GetSuccessor(parent->GetState(), action), parent, action, action_cost);

      if (Prune(successor)) {
        delete successor;
        continue;
      }

      auto duplicate_open = FindNodeInOpen(successor);
      if (duplicate_open) {
        std::cout << *successor->GetState() << " already in open" << std::endl;
        duplicate_open->AddParent(parent, action->Clone(), action_cost);
        ManageDuplicateInOpen(duplicate_open);
        parent->AddSuccessor(duplicate_open);
        delete successor;
        continue;
      }

      auto duplicate_close = FindNodeInClose(successor);
      if (duplicate_close) {
        std::cout << *successor->GetState() << " already in close" << std::endl;
        duplicate_close->AddParent(parent, action->Clone(), action_cost);
        ManageDuplicateInOpen(duplicate_close);
        parent->AddSuccessor(duplicate_close);
        delete successor;
        auto connected_goal = duplicate_close->GetConnectedGoal();
        if (connected_goal)
          std::cout << *duplicate_close->GetState() << " connected to goal " << *connected_goal->GetState()
                    << std::endl;
        if (connected_goal && GetPlan(plan, connected_goal)) {
          return connected_goal;
        }
        continue;
      }

      
      parent->AddSuccessor(successor);    
      if (problem_->IsGoal(successor->GetState())) {
        successor->SetConnectedGoal(successor);
        AddToClose(successor);

        if (GetPlan(plan, successor)) {
          AddToClose(parent);
          return successor;
        }
      }else{
        AddToOpen(successor);
      }
    }
    AddToClose(parent);
    return NULL;
  };
  virtual void Clear(){};

  virtual void AddToOpen(Node *const node) = 0;
  virtual Node *const FindNodeInOpen(Node *const node) = 0;
  virtual void ManageDuplicateInOpen(Node *const node){
      // In case you need for instance reorder open list after a duplicate has appeared
  };
  virtual Node *const ExtractNode() = 0; // Select Node, delete node from Open

  virtual void AddToClose(Node *const node) = 0;
  virtual Node *const FindNodeInClose(Node *const node) = 0;
  virtual void ManageDuplicateInClose(Node *const node){
      // In case you need for instance reorder open list after a duplicate has appeared
  };

  virtual bool Prune(Node *const node) const { return false; }

  virtual bool GetPlan(Plan &plan, Node *const goal) {
    std::cout << "Starting plan computation from goal " << *goal->GetState() << std::endl;
    plan.clear();

    Node *current_node = goal;
    plan.states.push_back(current_node->GetState()->Clone());
    plan.total_cost = current_node->GetAccumulatedCost();

    while (current_node != root_node_) {
      auto action = current_node->GetAction();
      auto parent = current_node->GetParent();

      assert(action && parent);

      if (current_node->IsParentConfirmed() || problem_->IsActionValid(parent->GetState(), action, false)) {
        if (!current_node->IsParentConfirmed()) {
          current_node->ConfirmParent();
        } else {
          std::cout << *action << " from " << *parent->GetState() << " already confirmed" << std::endl;
        }
        plan.actions.push_back(action->Clone());
        plan.costs.push_back(current_node->GetActionCost());

        current_node = parent;
        current_node->SetConnectedGoal(goal);
        plan.states.push_back(current_node->GetState()->Clone());
      } else {
        std::cout << "Removing parent" << std::endl;
        current_node->RemoveParent();
        if (isfinite(goal->GetAccumulatedCost())) {
          // If goal still has finite accumulated cost, it is still connected to root_node
          return GetPlan(plan, goal);
        } else {
          return false;
        }
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