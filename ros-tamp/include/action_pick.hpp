#pragma once
#include <Eigen/Geometry>
#include <action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionIK.h>

class PickAction : public Action {
public:
  PickAction(const std::string &target_object_id, const moveit_msgs::RobotState::_joint_state_type &joint_goal,
             const Eigen::Affine3d *selected_grasp, const Eigen::Affine3d &target_object_pose,
             moveit::planning_interface::MoveGroupInterface::Plan to_object_plan =
                 moveit::planning_interface::MoveGroupInterface::Plan(),
             moveit::planning_interface::MoveGroupInterface::Plan to_home_plan =
                 moveit::planning_interface::MoveGroupInterface::Plan())
      : Action(), action_id_("PICK"), target_object_id_(target_object_id), joint_goal_(joint_goal),
        selected_grasp_(selected_grasp), target_object_pose_(target_object_pose), to_object_plan_(to_object_plan),
        to_home_plan_(to_home_plan){};

  virtual Action *Clone() const override {
    return new PickAction(target_object_id_, joint_goal_, selected_grasp_, target_object_pose_,to_object_plan_, to_home_plan_);
  };

  void print(std::ostream &os) const override {

    // os<< action_id_ <<" to position: "<< target_location_.translation().transpose()<<" and orientation:
    // "<<target_location_.rotation().transpose();
    os << action_id_ << " object: " << target_object_id_;
  }

  void SetToObjectPlan(const moveit::planning_interface::MoveGroupInterface::Plan &to_object_plan) {
    to_object_plan_ = to_object_plan;
  }
  void SetToHomePlan(const moveit::planning_interface::MoveGroupInterface::Plan &to_home_plan) {
    to_home_plan_ = to_home_plan;
  }
  moveit::planning_interface::MoveGroupInterface::Plan GetToObjectPlan() const { return to_object_plan_; }
  moveit::planning_interface::MoveGroupInterface::Plan GetToHomePlan() const { return to_home_plan_; }

  std::string GetActionId() const { return action_id_; }

  std::string GetTargetObjectId() const { return target_object_id_; }

  const Eigen::Affine3d *GetGrasp() const { return selected_grasp_; }
  Eigen::Affine3d GetTargetObjectPose() const { return target_object_pose_; }

  moveit_msgs::RobotState::_joint_state_type GetJointGoal() const { return joint_goal_; }

protected:
  std::string action_id_;
  std::string target_object_id_;
  moveit_msgs::RobotState::_joint_state_type joint_goal_;
  const Eigen::Affine3d *selected_grasp_;
  Eigen::Affine3d target_object_pose_; // Object pose to be picked in world reference (it is not the pose to send to the
                                       // IK solver for grasp it). It is used when checking the whole plan existance (no
                                       // lazy valid action check) for setting a collision object.
  moveit::planning_interface::MoveGroupInterface::Plan to_object_plan_;
  moveit::planning_interface::MoveGroupInterface::Plan to_home_plan_;
};