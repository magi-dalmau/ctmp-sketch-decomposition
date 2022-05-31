#include <brfs.hpp>
#include <iostream>
#include <random>
#include <ros/ros.h>


template <class T> inline void hash_combine(std::size_t &s, const T &v) {
  std::hash<T> h;
  s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
}

class MyState : public State {
public:
  MyState(std::size_t i, std::size_t j) : State(), i_(i), j_(j){};

  virtual std::size_t GetHash() const {
    std::size_t hash = 0;
    hash_combine(hash, i_);
    hash_combine(hash, j_);
    return hash;
  }

  virtual State *Clone() const { return new MyState(i_, j_); }

  void print(std::ostream &os) const override {
    os << "(" << i_ << ", " << j_ << ")";
  }

  std::size_t i_, j_;
};

class MyAction : public Action {
public:
  enum Type { UP, DOWN, RIGHT, LEFT };
  Type type_;
  MyAction(Type type) : Action(), type_(type){};
  virtual Action *Clone() const { return new MyAction(type_); };
  void print(std::ostream &os) const override {
    switch (type_)
    {
    case UP:
      os << "UP";
      break;
    case DOWN:
      os << "DOWN";
      break;
    case LEFT:
      os << "LEFT";
      break;
    case RIGHT:
      os << "RIGHT";
      break;
    default:
      break;
    }
  }
};

class MyProblem : public Problem {
public:
  MyProblem(std::size_t num_rows, std::size_t num_cols, std::vector<std::pair<std::size_t, std::size_t>> obs_coords,
            std::pair<std::size_t, std::size_t> start_coords,
            std::vector<std::pair<std::size_t, std::size_t>> goal_coords, double lazy_prob)
      // lazy_prob probability of non-lazily checking an action even lazy flag is enable
      : Problem(), num_rows_(num_rows), num_cols_(num_cols), obs_coords_(obs_coords), start_coords_(start_coords),
        goal_coords_(goal_coords), gen(static_cast<long unsigned int>(time(0))), dist(lazy_prob) {
    actions_.push_back(new MyAction(MyAction::UP));
    actions_.push_back(new MyAction(MyAction::DOWN));
    actions_.push_back(new MyAction(MyAction::RIGHT));
    actions_.push_back(new MyAction(MyAction::LEFT));
  };

  virtual State *const Start() const { return new MyState(start_coords_.first, start_coords_.second); };

  virtual bool IsGoal(State const *const state) const {
    const MyState *my_state = dynamic_cast<const MyState *>(state);
    if (!my_state) {
      std::cout << __FILE__ << ":" << __LINE__ << std::endl;
      return false;
    }

    for (auto coords : goal_coords_) {
      if (my_state->i_ == coords.first && my_state->j_ == coords.second) {
        std::cout << *state << " is goal" << std::endl;
        return true;
      }
    }

    std::cout << *state << " not goal" << std::endl;
    return false;
    };

  virtual bool IsActionValid(State const *const state, Action const *const action, bool lazy = false) {
    auto successor = GetSuccessor(state, action);
    const MyState *my_state = dynamic_cast<const MyState *>(successor);
    if (!my_state) {
      std::cout << *action << " from " << *state << " not valid" << std::endl;
      return false;
    }

    if (!lazy || dist(gen)) {
      for (auto coords : obs_coords_) {
        if (my_state->i_ == coords.first && my_state->j_ == coords.second) {
          std::cout << *action << " from " << *state << " collides" << std::endl;
          delete successor;
          return false;
        }
      }
    }
    delete successor;
    std::cout << *action << " from " << *state << " is valid" << std::endl;
    return true;
  };

  virtual State *const GetSuccessor(State const *const state, Action const *const action) {
    const MyState *my_state = dynamic_cast<const MyState *>(state);
    if (!my_state) {
      std::cout << __FILE__ << ":" << __LINE__ << std::endl;
      return nullptr;
    }

    const MyAction *my_action = dynamic_cast<const MyAction *>(action);
    if (!my_action) {
      std::cout << __FILE__ << ":" << __LINE__ << std::endl;
      return nullptr;
    }

    switch (my_action->type_) {
    case MyAction::RIGHT:
      if (my_state->j_ + 1 < num_cols_)
        return new MyState(my_state->i_, my_state->j_ + 1);
      break;
    case MyAction::LEFT:
      if (my_state->j_ > 0)
        return new MyState(my_state->i_, my_state->j_ - 1);
      break;
    case MyAction::DOWN:
      if (my_state->i_ + 1 < num_rows_)
        return new MyState(my_state->i_ + 1, my_state->j_);
      break;
    case MyAction::UP:
      if (my_state->i_ > 0)
        return new MyState(my_state->i_ - 1, my_state->j_);
      break;

    default:
      break;
    }
    return nullptr;
  };

  void print(std::ostream &os) const override {
    std::string problem_str;
    InitEmptyWorld(problem_str);
    AddObjectToWorld(problem_str, ObstacleSymbol(), obs_coords_);
    AddObjectToWorld(problem_str, GoalSymbol(), goal_coords_);
    AddObjectToWorld(problem_str, AgentSymbol(), start_coords_.first, start_coords_.second);

    os << linearStringToGrid(problem_str);
  }
  std::string AgentSymbol() const { return "@"; };
  std::string EmptySymbol() const { return "O"; };
  std::string ObstacleSymbol() const { return "#"; };
  std::string GoalSymbol() const { return "$"; };

  void InitEmptyWorld(std::string &problem_str) const {
    problem_str = "";
    for (std::size_t i = 0; i < num_rows_; i++) {
      for (std::size_t j = 0; j < num_cols_; j++) {
        problem_str += EmptySymbol();
      }
    }
  }
  void AddObjectToWorld(std::string &problem_str, const std::string &object, const std::size_t coord_i,
                        const std::size_t coord_j) const {
    problem_str.replace(vectorizeIndex(coord_i, coord_j), 1, object);
  };
  void AddObjectToWorld(std::string &problem_str, const std::string &object,
                        const std::vector<std::pair<std::size_t, std::size_t>> &coords) const {
    for (const auto coord : coords) {
      AddObjectToWorld(problem_str, object, coord.first, coord.second);
    }
  };

  std::size_t vectorizeIndex(const std::size_t i, const std::size_t j) const { return i * num_cols_ + j; };
  std::string linearStringToGrid(const std::string &problem_string) const {
    std::string grid_string="";
    for (std::size_t i=0; i<num_rows_;i++){
      grid_string+=problem_string.substr(vectorizeIndex(i,0),num_cols_);
      grid_string+="\n";
    }
    return grid_string;
  }

protected:
  const std::size_t num_rows_;
  const std::size_t num_cols_;
  const std::vector<std::pair<std::size_t, std::size_t>> obs_coords_;
  const std::pair<std::size_t, std::size_t> start_coords_;
  const std::vector<std::pair<std::size_t, std::size_t>> goal_coords_;
  std::mt19937 gen;
  std::bernoulli_distribution dist;
};

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "test_bfrs_node");
  ROS_INFO("Starting Brfs test");
    // Declare Node Handle
  ros::NodeHandle nh("~");
  bool lazy=nh.param("lazy",true);


  auto problem = new MyProblem(5, 5, {{1, 2}, {2, 2}}, {0, 2}, {{2, 1}}, 0.0);
  std::cout << "My problem is:\n" << *problem << std::endl;

  auto brfs = new BrFS(problem);

  Plan plan;
  if (brfs->Solve(plan, lazy)) {
    std::cout << "Found solution" << std::endl;
    std::cout << plan << std::endl;
  } else {
    std::cout << "Solution not found" << std::endl;
  }

  return 0;
}