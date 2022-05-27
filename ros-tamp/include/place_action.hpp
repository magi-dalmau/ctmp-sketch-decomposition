#pragma once
#include <Eigen/Geometry>
#include <action.hpp>
#include <moveit_msgs/GetPositionIK.h>

class PlaceAction : public Action {
public:
  PlaceAction(const std::string &target_object_id, const moveit_msgs::RobotState::_joint_state_type &joint_goal)
      : Action() {
    action_id_ = "PLACE";
    target_object_id_ = target_object_id;
    joint_goal_ = joint_goal;
  };

  virtual Action *Clone() const { return new PlaceAction(target_object_id_,joint_goal_); };

  void print(std::ostream &os) const override {

    // os<< action_id_ <<" to position: "<< target_location_.translation().transpose()<<" and orientation:
    // "<<target_location_.rotation().transpose();
    os << action_id_ << " object: " << target_object_id_;
  }

  std::string GetActionId() const { return action_id_; }

protected:
  std::string action_id_;
  std::string target_object_id_;
  // Eigen::Affine3d target_location_;
  moveit_msgs::RobotState::_joint_state_type joint_goal_;
};