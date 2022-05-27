// ros_tamp
#include <move_base_action.hpp>
#include <place_action.hpp>
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

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Generic CPP
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

class MoveitTampProblem : public Problem {

public:
  MoveitTampProblem(const std::string &filename, const std::string &planning_group, ros::NodeHandle *nodehandle);
  virtual State *const Start() { return nullptr; };
  virtual bool IsGoal(State const *const state) const { return false; };
  virtual std::vector<Action *> GetValidActions(State const *const state, bool lazy = false) {
    return std::vector<Action *>();
  }

  virtual ~MoveitTampProblem();

  virtual std::vector<Action *> GetValidActions(State const *const state,
                                                bool lazy = false) override; // overriding voluntarily

  virtual bool IsActionValid(State const *const state, Action const *const action, bool lazy = false) {
    // Checks always:
    // Free gripper/grasped object and a grasp (if needed)
    // In workspace (i.e. within a sphere)
    // IK solution exists (collision free) for pick & parking pose or place pose
    // Pick/place/Home configuration collision-free

    if (!lazy) {
      // In addition, checks if non-lazy:
      // Plans the trajectory
    }

    return true;
  };

  virtual State *const GetSuccessor(State const *const state, Action const *const action) { return nullptr; };
  // virtual double GetCost(State const *const state, Action const *const action) { return 1.; };

  virtual void print(std::ostream &os) const override{};

  virtual std::size_t GetNovelty(State const *const state) const override { return 0; };

protected:
  // METHODS
  // initializations
  void LoadWorld(const std::string &filename);

  // Problem actions related
  MoveitTampState ApplyPlaceAction(const MoveitTampState &current_state, const std::string &obj_id,
                                   const geometry_msgs::Pose &placement,
                                   const const geometry_msgs::Pose &stable_object_pose) {
    MoveitTampState new_state(current_state); // TODO, aixi o es pot reaprofitar mmilor el current state

    // Compute final object pose
    geometry_msgs::PoseStamped final_pose;
    final_pose.header.frame_id = common_reference_; // TODO Així o associem a una surface?
    // Add to state new object pose and updated robot state (gripper)
    // Detach collision object
    DetachCollisionObject(obj_id, final_pose);
  }

  // Manipulator movements related
  bool ComputeIK(const geometry_msgs::Pose &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal);
  bool ComputeIK(const Eigen::Affine3d &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal);

  bool PlanToJoinTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan);
  void InitIKRequest(moveit_msgs::GetPositionIK &srv); // use a explicit srv reference
  void InitIKRequest();                                // use directly the class member srv_
  bool ExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan);
  // Collision object related
  void MoveCollisionObjects(const std::vector<std::string> &obj_ids, const std::vector<geometry_msgs::Pose> &new_poses,
                            const std::vector<std::string> &new_reference_frames);
  void MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose,
                           const std::string &new_reference_frame);
  bool AttachCollisionObject(const std::string &obj_id, const geometry_msgs::PoseStamped &grasping_pose);
  bool DetachCollisionObject(const std::string &obj_id, const geometry_msgs::PoseStamped &placement_pose);
  bool OnWorkspace(const Eigen::Affine3d &robot_base_pose, const Eigen::Affine3d &target_pose) const;

  moveit_msgs::CollisionObject GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                              const geometry_msgs::Pose &new_pose,
                                                              const std::string &new_reference_frame);
  shape_msgs::Mesh MeshMsgFromFile(const std::string &mesh_path);
  void PopulateLocationsConnections();
  bool LocationReachable(const Eigen::Affine3d &origin, const Eigen::Affine3d &destination);

  // visualization
  void Publish(const ros::TimerEvent &event);

  // Test only
  void AddTestCollision(); // TEST ONLY

  // Classes
  class BaseStateSpace {
  public:
    bool isValid(double x, double y, double yaw) const {
      return isfinite(x) && (x >= x_min) && (x <= x_max) && isfinite(y) && (y >= y_min) && (y <= y_max) &&
             isfinite(yaw);
    }
    double x_min, y_min, x_max, y_max;
  };

  class SupportingSurface {
  public:
    SupportingSurface(double x_min, double x_max, double y_min, double y_max, const Eigen::Affine3d &surface_pose);
    Eigen::Affine3d pose_;
    Eigen::Vector2d min_, max_;
    std::vector<Eigen::Affine3d> placements_;
    double area() const;
    bool on(const Eigen::Vector3d &position) const;
    void sample();
  };

  class Object {
  public:
    std::string name_;
    std::string mesh_;
    Eigen::Affine3d pose_;
    bool moveable_;
    std::vector<SupportingSurface> surfaces_;
    std::vector<Eigen::Affine3d> grasps_;
    std::vector<Eigen::Affine3d> placements_;
  };

  // OBJECTS
  ros::NodeHandle nh_;
  ros::Publisher pub_placements_;
  ros::Publisher pub_locations_;
  ros::Publisher pub_objects_;
  ros::Timer display_timer_;
  geometry_msgs::PoseArray display_placements_;
  geometry_msgs::PoseArray display_locations_;
  visualization_msgs::MarkerArray display_objects_;
  std::map<std::string, Object> objects_;
  std::vector<Eigen::Affine3d> base_locations_;
  std::unordered_map<std::size_t, std::vector<std::size_t>> locations_connections_; // TODO: Margin to optimize it?

  double allowed_distance_between_connected_locations_;
  Eigen::Affine3d robot_origin_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient ik_service_client_;
  std::vector<std::string> group_joint_names_;
  std::size_t num_group_joints_;
  std::string robot_root_tf_;
  std::string common_reference_;
  moveit_msgs::GetPositionIK srv_;
  std::default_random_engine generator_;
};
