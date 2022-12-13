#pragma once
#include <moveit_tamp_problem.hpp>

class BlocksWorldTampProblem : public MoveitTampProblem {
public:
  /*


   */
  BlocksWorldTampProblem(const std::string &filename, const std::string &planning_group, ros::NodeHandle *nodehandle);
  // void AdaptativeSampling(const State *const state);

protected:
  /*

   */
  // virtual void LoadWorld(const std::string &filename) override;
  virtual void ComputeStateSketchFeatures(State *const state) override;
  virtual void SetMisplacedObjects(MoveitTampState *const state) const override;

  virtual bool Misplaced(const std::string &name, const Eigen::Affine3d &pose) const override;
  // std::size_t BlockingObjects(MoveitTampState const *const state, const Eigen::Affine3d &robot_pose,
  //                             const Eigen::Affine3d &gripper_pose, const std::string &misplaced_object_name,
  //                             const std::size_t max_number) const;
  // std::size_t BlockingObjectsPick(MoveitTampState const *const state, const Eigen::Affine3d &object_pose,
  //                                 const Eigen::Affine3d &gripper_pose, const std::string &misplaced_object_name,
  //                                 const std::size_t max_number) const;
  // std::size_t BlockingObjectsPlace(MoveitTampState const *const state, const Eigen::Affine3d &placement_pose,
  //                                  const Eigen::Affine3d &grasp, const std::vector<Eigen::Affine3d> &sops,
  //                                  const std::string &misplaced_object_name,
  //                                  const std::size_t max_num_blocking_objects) const;
  // virtual void SetBlockingObjects(MoveitTampState *const state, bool compute_s = false) const override;

  // Objects  
  double goal_tolerance_radius_;
  bool only_green_goals_;
};