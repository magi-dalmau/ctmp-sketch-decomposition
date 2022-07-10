
#pragma once
#include <moveit_tamp_problem.hpp>

class ClutteredTampProblem : public MoveitTampProblem {
public:
  /*


   */
  ClutteredTampProblem(const std::string &filename, const std::string &planning_group, ros::NodeHandle *nodehandle);
  // void AdaptativeSampling(const State *const state);

protected:
  /*

   */
  //   virtual void ComputeStateSketchFeatures(State *const state) const override;
  //   virtual void SetMisplacedObjects(MoveitTampState *const state) const override;
  virtual void SetMisplacedObjects(MoveitTampState *const state) const override;

  virtual bool Misplaced(const std::string &name, const Eigen::Affine3d &pose) const override;
  // virtual void SetBlockingObjects(MoveitTampState *const state, bool compute_s = false) const override;
  // std::size_t BlockingObjectsPlace(MoveitTampState const *const state, const Eigen::Affine3d &placement_pose,
  //                                  const Eigen::Affine3d &grasp, const std::vector<Eigen::Affine3d> &sops,
  //                                  const std::string &misplaced_object_name,
  //                                  const std::size_t max_num_blocking_objects) const;

  // std::size_t BlockingObjectsPick(MoveitTampState const *const state, const Eigen::Affine3d &object_pose,
  //                                 const Eigen::Affine3d &gripper_pose, const std::string &misplaced_object_name,
  //                                 const std::size_t max_num_blocking_objects) const;

  // std::size_t BlockingObjects(MoveitTampState const *const state, const Eigen::Affine3d &target_pose,
  //                             const Eigen::Affine3d &robot_pose, const Eigen::Affine3d &gripper_pose,
  //                             const std::string &misplaced_object_name,
  //                             const std::size_t max_num_blocking_objects) const;
  // OBJECTS
  std::map<std::string, std::string> region_goals_;
  // bool goal_region_;
};