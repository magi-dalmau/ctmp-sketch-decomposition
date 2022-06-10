#pragma once
// ros_tamp
#include <state.hpp>
#include <utils.hpp>

// ROS
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

// Generic CPP
#include <iostream>

class MoveitTampState : public State {
  // struct RobotData {
  //   geometry_msgs::Pose base_pose;
  //   std::vector<double> joint_positions;
  // };

public:
  MoveitTampState(Eigen::Affine3d base_pose, std::vector<Eigen::Affine3d> object_poses, std::size_t hash,
                  std::vector<std::size_t> features, const std::string attached_obj_id = "",
                  const Eigen::Affine3d *selected_grasp = nullptr)
      : robot_base_pose_(base_pose), object_poses_(object_poses), state_hash_(hash), features_(features),
        object_attached_(attached_obj_id), selected_grasp_(selected_grasp){

                                           };

  virtual std::size_t GetHash() const override { return state_hash_; }
  // void CombineHashBasePose(std::size_t &hash) const { CombineHashPose(hash, robot_base_pose_); }
  // void CombineHashObjectsData(std::size_t &hash) const {
  //   for (const auto pose : object_poses_) {
  //     CombineHashPose(hash, pose);
  //   };
  // };

  virtual State *Clone() const override {
    return new MoveitTampState(robot_base_pose_, object_poses_, state_hash_, features_, object_attached_,
                               selected_grasp_);
  }

  virtual std::vector<std::size_t> GetFeatures() const override { return features_; };

  virtual double distance(const State *const state) const override {
    auto casted_state = dynamic_cast<const MoveitTampState *const>(state);
    double dist = (robot_base_pose_.matrix() - casted_state->GetRobotBasePose().matrix()).squaredNorm();
    for (std::size_t i = 0; i < object_poses_.size(); ++i) {
      dist += (object_poses_.at(i).matrix() - casted_state->GetObjectPoses().at(i).matrix()).squaredNorm();
    }

    return dist;
  }

  bool HasObjectAttached() const { return !object_attached_.empty(); };
  Eigen::Affine3d GetRobotBasePose() const { return robot_base_pose_; }
  std::vector<Eigen::Affine3d> GetObjectPoses() const { return object_poses_; }
  std::string GetAttatchedObject() const { return object_attached_; }
  const Eigen::Affine3d *GetGrasp() const { return selected_grasp_; }

protected:
  // METHODS

  void Affine3dToString(std::ostream &os, const Eigen::Affine3d &pose) const {
    os << "position: " << pose.translation().transpose()
       << "\torientation: " << Eigen::Quaterniond(pose.rotation()).coeffs().transpose() << std::endl;
  }

  // std::string PoseToString(const geometry_msgs::Pose &pose) const {
  //   std::string str = "";
  //   str += "position( " + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " +
  //          std::to_string(pose.position.z);
  //   str += "\torientation( " + std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", "
  //   +
  //          std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w);
  //   return str;
  // }

  void print(std::ostream &os) const override {
    GetRobotDataString(os);
    GetObjectsDataString(os);
  }

  void GetRobotDataString(std::ostream &os) const { Affine3dToString(os, robot_base_pose_); }

  void GetObjectsDataString(std::ostream &os) const {
    os << "\n Object positions:";
    // std::cout << "there are " << object_poses_.size() << " objects" << std::endl;
    for (size_t i = 0; i < object_poses_.size(); i++) {
      os << "\nObject " << std::to_string(i) << " : ";
      Affine3dToString(os, object_poses_.at(i));
    }
  }

  // OBJECTS

  const Eigen::Affine3d robot_base_pose_;
  const std::vector<Eigen::Affine3d> object_poses_;
  std::size_t state_hash_;
  std::vector<std::size_t> features_;
  const std::string object_attached_;
  const Eigen::Affine3d *selected_grasp_; // Not considered for hash, only a helper info
  // std::vector<moveit_msgs::CollisionObject> object_poses_;
};