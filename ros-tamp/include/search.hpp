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

  virtual bool Solve(Plan &plan, bool lazy = false, const State *const start = nullptr) {
    std::cout << "Starting solver" << std::endl;
    Clear();
    root_node_ = new Node(start ? start->Clone() : problem_->Start());
    num_open_++;
    AddToOpen(root_node_);
    auto node = ExtractNode();
    if (node)
      num_open_--;

    if (problem_->IsGoal(node->GetState())) {
      GetPlan(plan, node);
      printStatistics();
      return true;
    }

    while (node) {
      if (num_processed_ % 100 == 0) {
        printStatistics();
      }

      num_processed_++;
      auto end = Process(node, plan, lazy);

      if (end) {
        printStatistics();
        return true;
      }

      node = ExtractNode();
      if (node)
        num_open_--;
    }

    printStatistics();
    return false;
  }

protected:
  virtual void printStatistics() const {
    std::cout << "Open: " << num_open_ << " Dup-Open: " << num_duplicates_open_ << " Closed: " << num_closed_
              << " Dup-Closed: " << num_duplicates_closed_ << " Pruned: " << num_pruned_
              << " Dup-Pruned: " << num_duplicates_pruned_ << " Processed: " << num_processed_
              << " Rewired: " << num_rewired_ << " Get Plan: " << num_plans_
              << " Already confirmed: " << num_already_confirmed_ << std::endl;
    // std::string x;
    problem_->PrintStatistics();
  }

  virtual Node *const Process(Node *const parent, Plan &plan, bool lazy) {
    // std::cout << "Starting processing " << *parent->GetState() << std::endl;
    for (const auto action : problem_->GetValidActions(parent->GetState(), lazy)) {
      double action_cost = problem_->GetCost(parent->GetState(), action);
      auto successor =
          new Node(problem_->GetSuccessor(parent->GetState(), action), parent, action->Clone(), action_cost);

      assert(successor->GetState()->GetHash() != parent->GetState()->GetHash());

      auto duplicate_open = FindNodeInOpen(successor);
      if (duplicate_open) {
        num_duplicates_open_++;
        // std::cout << *successor->GetState() << " already in open" << std::endl;
        duplicate_open->AddParent(parent, action->Clone(), action_cost);
        ManageDuplicateInOpen(duplicate_open);
        parent->AddSuccessor(duplicate_open);
        delete successor;
        continue;
      }

      auto duplicate_close = FindNodeInClose(successor);
      if (duplicate_close) {
        num_duplicates_closed_++;
        // std::cout << *successor->GetState() << " already in close" << std::endl;
        auto rewired = duplicate_close->AddParent(parent, action->Clone(), action_cost);
        ManageDuplicateInClose(duplicate_close);
        parent->AddSuccessor(duplicate_close);
        delete successor;
        if (rewired) {
          num_rewired_++;
          auto connected_goal = duplicate_close->GetConnectedGoal();
          if (connected_goal) {
            std::cout << "Fond a connected to goal node" << std::endl;
            std::cout << *duplicate_close->GetState() << " connected to goal " << *connected_goal->GetState()
                      << std::endl;
          }
          if (connected_goal && GetPlan(plan, connected_goal)) {
            num_closed_++;
            AddToClose(parent);
            return connected_goal;
          }
        }
        continue;
      }

      auto duplicate_pruned = FindNodeInPruned(successor);
      if (duplicate_pruned) {
        num_duplicates_pruned_++;
        // std::cout << *successor->GetState() << " already in open" << std::endl;
        duplicate_pruned->AddParent(parent, action->Clone(), action_cost);
        ManageDuplicateInPruned(duplicate_pruned);
        parent->AddSuccessor(duplicate_pruned);
        delete successor;
        continue;
      }

      parent->AddSuccessor(successor);
      if (problem_->IsGoal(successor->GetState())) {
        successor->SetConnectedGoal(successor);
        num_closed_++;
        AddToClose(successor);

        if (GetPlan(plan, successor)) {
          num_closed_++;
          AddToClose(parent);
          return successor;
        }
      } else if (Prune(successor)) {
        num_pruned_++;
        ManagePruned(successor);
      } else {
        num_open_++;
        AddToOpen(successor);
      }
    }
    num_closed_++;
    AddToClose(parent);
    return nullptr;
  };
  virtual void Clear() {
    num_open_ = 0;
    num_duplicates_open_ = 0;
    num_closed_ = 0;
    num_duplicates_closed_ = 0;
    num_pruned_ = 0;
    num_duplicates_pruned_ = 0;
    num_processed_ = 0;
    num_rewired_ = 0;
    num_plans_ = 0;
    num_already_confirmed_ = 0;
  };

  virtual void AddToOpen(Node *const node) = 0;
  virtual Node *const FindNodeInOpen(Node *const node) = 0;
  virtual void ManageDuplicateInOpen(Node *const node){
      // In case you need for instance reorder open list after a duplicate has appeared
  };
  virtual Node *const ExtractNode() = 0; // Select Node, delete node from Open

  virtual void AddToClose(Node *const node) = 0;
  virtual Node *const FindNodeInClose(Node *const node) = 0;
  virtual void ManageDuplicateInClose(Node *const node){
      // In case you need for instance reorder close list after a duplicate has appeared
  };
  virtual Node *const FindNodeInPruned(Node *const node) { return nullptr; };
  virtual void ManageDuplicateInPruned(Node *const node){
      // In case you need for instance reorder pruned list after a duplicate has appeared
  };
  virtual bool Prune(Node *const node) { return false; }
  virtual void ManagePruned(Node *const node) {
    std::cout << "ManagePruned Search" << std::endl;
    delete node;
  }
  virtual void ManageOrphan(Node *const node){};

  virtual bool GetPlan(Plan &plan, Node *const goal) {
    num_plans_++;
    // std::cout << "Starting plan computation from goal " << *goal->GetState() << std::endl;
    std::cout << "Starting plan computation" << std::endl;
    // std::string x;
    // std::cin >> x;

    plan.clear();

    Node *current_node = goal;
    plan.states.push_back(current_node->GetState()->Clone());
    plan.total_cost = current_node->GetAccumulatedCost();

    while (current_node != root_node_) {
      auto action = current_node->GetAction()->Clone();
      auto parent = current_node->GetParent();

      assert(action && parent);

      if (current_node->IsParentConfirmed() || problem_->IsActionValid(parent->GetState(), action, false)) {
        if (!current_node->IsParentConfirmed()) {
          current_node->ConfirmParent();
        } else {
          num_already_confirmed_++;
          std::cout << *action << " from " << *parent->GetState() << " already confirmed" << std::endl;
        }
        plan.actions.push_back(action);
        plan.costs.push_back(current_node->GetActionCost());

        current_node = parent;
        current_node->SetConnectedGoal(goal);
        plan.states.push_back(current_node->GetState()->Clone());
      } else {
        delete action;

        std::cout << "Removing parent" << std::endl;
        current_node->RemoveParent();
        if (!isfinite(current_node->GetAccumulatedCost())) {
          ManageOrphan(current_node);
        }
        if (isfinite(goal->GetAccumulatedCost())) {
          num_rewired_++;
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
  // statistics
  std::size_t num_open_, num_duplicates_open_, num_closed_, num_duplicates_closed_, num_pruned_, num_processed_,
      num_rewired_, num_plans_, num_already_confirmed_, num_duplicates_pruned_;
};