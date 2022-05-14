// ros_tamp
#include <moveit_tamp_action.hpp>
#include <moveit_tamp_state.hpp>
#include <problem.hpp>

// ROS
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>

#include <ros/ros.h>
// Generic CPP
#include <iostream>

class MoveitTampProblem : public Problem {

public:
  MoveitTampProblem(const std::string &planning_group, ros::NodeHandle *nodehandle);
  virtual State *const Start() {return nullptr;};
  virtual bool IsGoal(State const *const state) const {return false;};
  virtual std::vector<Action *> GetValidActions(State const *const state, bool lazy = false) {return std::vector<Action *>(); }

  virtual bool IsActionValid(State const *const state, Action const *const action, bool lazy = false) {
    // Checks always:
    // Free gripper/grasped object and a grasp (if needed)
    // In workspace (i.e. within a sphere)
    // (End-effector collision-free)
    // IK solution exists
    // Pick/place/Home configuration collision-free
    
    if(!lazy){
      // In addition, checks if non-lazy:
      // Plans the trajectory
      
    }

    return true;
  };
  virtual State *const GetSuccessor(State const *const state, Action const *const action){ return nullptr;};
  // virtual double GetCost(State const *const state, Action const *const action) { return 1.; };

  virtual void print(std::ostream &os) const override{};

  virtual std::size_t GetNovelty(State const *const state) { return 0; };

protected:
  // METHODS

  bool ComputeIK(const geometry_msgs::Pose &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal);

  bool PlanToJoinTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan);
  void InitIKRequest(moveit_msgs::GetPositionIK &srv); // use a explicit srv reference
  void InitIKRequest();                                // use directly the class member srv_

  bool ExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan);
  void MoveCollisionObjects(const std::vector<std::string> &obj_ids, const std::vector<geometry_msgs::Pose> &new_poses);
  void MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose);
  moveit_msgs::CollisionObject GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                              const geometry_msgs::Pose &new_pose);

  void AddTestCollision(); // TEST ONLY

  // OBJECTS
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient ik_service_client_;
  std::vector<std::string> group_joint_names_;
  std::size_t num_group_joints_;
  std::string robot_root_tf_;
  moveit_msgs::GetPositionIK srv_;
};
