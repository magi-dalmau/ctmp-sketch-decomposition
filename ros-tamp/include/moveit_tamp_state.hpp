// ros_tamp
#include <state.hpp>

// ROS
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

// Generic CPP
#include <iostream>

template <class T> inline void hash_combine(std::size_t &s, const T &v) {
  std::hash<T> h;
  s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
}

class MoveitTampState : public State {
  // struct RobotData {
  //   geometry_msgs::Pose base_pose;
  //   std::vector<double> joint_positions;
  // };

public:
  MoveitTampState(Eigen::Affine3d base_pose, std::vector<Eigen::Affine3d> object_poses,
                  const std::string attached_obj_id = "", const Eigen::Affine3d *selected_grasp = nullptr)
      : robot_base_pose_(base_pose), object_poses_(object_poses), object_attached_(attached_obj_id),
        selected_grasp_(selected_grasp){

        };

  virtual std::size_t GetHash() const {
    std::size_t hash = 0;
    CombineHashBasePose(hash);
    CombineHashObjectsData(hash);
    return hash;
  }
  void CombineHashBasePose(std::size_t &hash) const { CombineHashPose(hash, robot_base_pose_); }
  void CombineHashObjectsData(std::size_t &hash) const {
    for (const auto pose : object_poses_) {
      CombineHashPose(hash, pose);
    };
  };
  static void CombineHashPose(std::size_t &hash, const Eigen::Affine3d &pose) {
    hash_combine<double>(hash, pose.translation()(0));
    hash_combine<double>(hash, pose.translation()(1));
    hash_combine<double>(hash, pose.translation()(2));
    Eigen::Quaterniond q(pose.rotation());
    // TODO solucionar doble mapeado quaternion
    hash_combine<double>(hash, q.w());
    hash_combine<double>(hash, q.x());
    hash_combine<double>(hash, q.y());
    hash_combine<double>(hash, q.z());
  }

  virtual State *Clone() const {
    return new MoveitTampState(robot_base_pose_, object_poses_, object_attached_, selected_grasp_);
  }

  bool HasObjectAttached() const { return !object_attached_.empty(); };
  Eigen::Affine3d GetRobotBasePose() const { return robot_base_pose_; }
  std::vector<Eigen::Affine3d> GetObjectPoses() const { return object_poses_; }
  std::string GetAttatchedObject() const { return object_attached_; }
  const Eigen::Affine3d *GetGrasp() const { return selected_grasp_; }

protected:
  // METHODS
  void Affine3dToString(std::ostream &os, const Eigen::Affine3d &pose) const {

    os << "position: " << pose.translation().transpose() << "\torientation: " << pose.rotation().transpose();
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
    // GetObjectsDataString(os);
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
  const std::string object_attached_;
  const Eigen::Affine3d *selected_grasp_; // Not considered for hash, only a helper info
  // std::vector<moveit_msgs::CollisionObject> object_poses_;
};
