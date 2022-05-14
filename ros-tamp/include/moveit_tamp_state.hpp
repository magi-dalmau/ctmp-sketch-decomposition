// ros_tamp
#include <state.hpp>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

// Generic CPP
#include <iostream>

template <class T> inline void hash_combine(std::size_t &s, const T &v) {
  std::hash<T> h;
  s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
}

class MoveitTampState : public State {
  struct RobotData {
    geometry_msgs::Pose base_pose;
    std::vector<double> joint_positions;
  };

public:
  MoveitTampState(geometry_msgs::Pose base_pose, std::vector<double> joint_positions,
                  std::vector<geometry_msgs::Pose> object_poses, const std::string attached_obj_id = "NONE")
      : robot_data_{base_pose, joint_positions}, object_attached_ (attached_obj_id){
    moveit_msgs::CollisionObject obj;
    obj.operation = moveit_msgs::CollisionObject::MOVE;
    obj.header.frame_id = ""; // TODO: Define/load frame reference for collision objects- maybe all with world ref
  };

  virtual std::size_t GetHash() const {
    std::size_t hash = 0;
    CombineHashRobotData(hash);
    CombineHashObjectsData(hash);
    return hash;
  }
  void CombineHashRobotData(std::size_t &hash) const {
    CombineHashPose(hash, robot_data_.base_pose);
    for (const auto position : robot_data_.joint_positions) {
      hash_combine<double>(hash, position);
    };
  }
  void CombineHashObjectsData(std::size_t &hash) const {
    for (const auto pose : object_poses_) {
      CombineHashPose(hash, pose);
    };
  };
  void CombineHashPose(std::size_t &hash, const geometry_msgs::Pose &pose) const {
    hash_combine<double>(hash, pose.position.x);
    hash_combine<double>(hash, pose.position.y);
    hash_combine<double>(hash, pose.position.z);
    hash_combine<double>(hash, pose.orientation.x);
    hash_combine<double>(hash, pose.orientation.y);
    hash_combine<double>(hash, pose.orientation.z);
    hash_combine<double>(hash, pose.orientation.w);
  }

  virtual State *Clone() const {
    return new MoveitTampState(robot_data_.base_pose, robot_data_.joint_positions, object_poses_);
  }

  void print(std::ostream &os) const override { os << GetRobotDataString() << GetObjectsDataString(); }

  std::string GetRobotDataString() const {
    std::string str = "";
    str += "\nRobot base pose: " + PoseToString(robot_data_.base_pose);
    str += "\nRobot joint positions:\n( ";
    for (const auto pos : robot_data_.joint_positions) {
      str += " " + std::to_string(pos) + ",";
    }
    str.pop_back();
    str += " )";
    return str;
  }

  std::string GetObjectsDataString() const {
    std::string str="";
    str+="\n Object positions:";
    for (size_t i = 0; i < object_poses_.size(); i++)
    {
      str+="\nObject "+std::to_string(i)+" : "+PoseToString(object_poses_.at(i));
    }
    return str;
  }

    std::string PoseToString(const geometry_msgs::Pose &pose) const {
      std::string str = "";
      str += "position( " + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " +
             std::to_string(pose.position.z);
      str += "\torientation( " + std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", " +
             std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w);
      return str;
    }

  protected:
    const RobotData robot_data_;
    const std::vector<geometry_msgs::Pose> object_poses_;
    const std::string object_attached_;
    // std::vector<moveit_msgs::CollisionObject> object_poses_;
    
  };
