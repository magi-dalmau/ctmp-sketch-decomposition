#pragma once
#include <Eigen/Geometry>
#include <action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionIK.h>

class PlaceAction : public Action {
public:
  PlaceAction(const std::string &target_object_id, const moveit_msgs::RobotState::_joint_state_type &joint_goal,
              const Eigen::Affine3d target_object_pose)
      : Action() {
    action_id_ = "PLACE";
    target_object_id_ = target_object_id;
    joint_goal_ = joint_goal;
    target_object_pose_ = target_object_pose;
  };

  virtual Action *Clone() const { return new PlaceAction(target_object_id_, joint_goal_, target_object_pose_); };

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

  std::string GetActionId() const { return action_id_; }
  std::string GetTargetObjectId() const { return target_object_id_; }
  const Eigen::Affine3d GetTargetObjectPose() const { return target_object_pose_; }
  moveit_msgs::RobotState::_joint_state_type GetJointGoal() const { return joint_goal_; }

protected:
  std::string action_id_;
  std::string target_object_id_;
  Eigen::Affine3d target_object_pose_;//This includes world to placement + placement to object (i.e. stable object pose)
  moveit_msgs::RobotState::_joint_state_type joint_goal_;
  moveit::planning_interface::MoveGroupInterface::Plan to_object_plan_;
  moveit::planning_interface::MoveGroupInterface::Plan to_home_plan_;
};