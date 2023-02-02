#pragma once
#include <moveit_tamp_problem.hpp>

class BlocksWorldTampProblem : public MoveitTampProblem {
public:
  /*


   */
  BlocksWorldTampProblem(const std::string &filename, const std::string &planning_group, ros::NodeHandle *nodehandle);
  // void AdaptativeSampling(const State *const state);

protected:
  struct CompareSequences {
    bool operator()(std::vector<std::size_t> a, std::vector<std::size_t> b) { return (a.size() > b.size()); }
  };

  virtual void ComputeStateSketchFeatures(State *const state) override;
  virtual void SetMisplacedObjects(MoveitTampState *const state) const override;
  virtual bool Misplaced(const std::string &name, const Eigen::Affine3d &pose) const override;
  bool MisplacedBlocks(const std::string &name, const Eigen::Affine3d &pose,
                       const std::vector<std::size_t> &longest_valid_seq) const;
  virtual bool AllGoalRegionBlockMisplaced(const std::string &object_name,
                                           MoveitTampState const *const state) const override;
  bool LoadAndSetGoals();
  bool IsNeighbour(const Eigen::Affine3d &pose_obj_left, const Eigen::Affine3d &pose_obj_right) const;
  bool IsValidSequence(const std::vector<std::size_t> &seq) const;
  std::string GetObjectId(const std::string &object_name) const;
  std::string GetObjectId(const std::size_t &object_index) const;
  std::string GetObjectClass(const std::string &object_name) const;
  std::string GetObjectClass(const std::size_t &object_index) const;

  // Objects
  double goal_tolerance_radius_;
  bool only_green_goals_;
  std::string goal_sequence_;
  std::map<std::size_t, std::pair<std::size_t, std::size_t>> object_neighbours_template_;
  std::vector<std::size_t> longest_valid_seq_;
  Eigen::Affine3d left2right_pose_;
};