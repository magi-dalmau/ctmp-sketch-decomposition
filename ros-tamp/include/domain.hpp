#include <vector>

class Action;

class State;

class Goal {
public:
  virtual bool isGoal(const State &state) = 0;
};

class Problem {
public:
  State *start;
  Goal *goal;
};

class Domain {
public:
  virtual std::vector<Action> getValidActions(const State &state) = 0;
  virtual State getSuccessor(const Action &action, const State &state) = 0;
  virtual double actionCost(const Action &action) { return 1; };
};