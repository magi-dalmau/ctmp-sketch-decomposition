#pragma once
#include <Eigen/Geometry>
#include <action.hpp>
#include <moveit_msgs/GetPositionIK.h>

class PickAction : public Action {
public:
  PickAction(const std::string &target_object_id, const moveit_msgs::RobotState::_joint_state_type &joint_goal,
             const Eigen::Affine3d *selected_grasp)
      : Action(), selected_grasp_(selected_grasp) {
    action_id_ = "PICK";
    target_object_id_ = target_object_id;
    joint_goal_ = joint_goal;
  };

  virtual Action *Clone() const { return new PickAction(target_object_id_, joint_goal_,selected_grasp_); };

  void print(std::ostream &os) const override {

    // os<< action_id_ <<" to position: "<< target_location_.translation().transpose()<<" and orientation:
    // "<<target_location_.rotation().transpose();
    os << action_id_ << " object: " << target_object_id_;
  }

  std::string GetActionId() const { return action_id_; }

  std::string GetTargetObjectId() const { return target_object_id_; }

  const Eigen::Affine3d *GetGrasp() const { return selected_grasp_; }

protected:
  std::string action_id_;
  std::string target_object_id_;
  const Eigen::Affine3d *selected_grasp_;
  moveit_msgs::RobotState::_joint_state_type joint_goal_;
};