// ros_tamp
#include <action_move_base.hpp>
#include <action_pick.hpp>
#include <action_place.hpp>
#include <moveit_tamp_state.hpp>
#include <problem.hpp>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// Generic CPP
#include <Eigen/Geometry>
#include <iostream>
#include <random>
#include <unordered_map>

class MoveitTampProblem : public Problem {
public:
  MoveitTampProblem(const std::string &filename, const std::string &planning_group, ros::NodeHandle *nodehandle);
  virtual State *const Start() const override;
  virtual bool IsGoal(State const *const state) const override;
  virtual ~MoveitTampProblem() override;

  // overriding GetValidActions voluntarily
  std::vector<Action *> GetValidActions(State const *const state, bool lazy = false) override;

  virtual bool IsActionValid(State const *const state, Action *const action, bool lazy = false) override;

  virtual State *const GetSuccessor(State const *const state, Action const *const action) override;

  virtual void print(std::ostream &os) const override{};
  virtual void PrintStatistics() const override;

  virtual bool ExecutePlan(const Plan &plan) override;

protected:
  // METHODS
  // initializations
  void LoadWorld(const std::string &filename);

  // // Problem actions related
  // MoveitTampState ApplyPlaceAction(const MoveitTampState &current_state, const std::string &obj_id,
  //                                  const geometry_msgs::Pose &placement,
  //                                  const geometry_msgs::Pose &stable_object_pose) {
  //   MoveitTampState new_state(current_state); current state

  //   // Compute final object pose
  //   geometry_msgs::PoseStamped final_pose;
  //   final_pose.header.frame_id = common_reference_;
  //   // Add to state new object pose and updated robot state (gripper)
  //   // Detach collision object
  //   DetachCollisionObject(obj_id, final_pose);
  // }
  bool SetActiveSketchRule(const State *const state) override;
  void ComputeStateSketchFeatures(State *const state) const;
  void SetMisplacedObjects(MoveitTampState *const state) const;
  bool Misplaced(const std::string &name, const Eigen::Affine3d &pose) const;
  void SetBlockingObjects(MoveitTampState *const state,bool compute_s = false) const;

  // Manipulator movements related
  bool ComputeIK(const geometry_msgs::Pose &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal,
                 const moveit_msgs::AttachedCollisionObject &attached_object);
  bool ComputeIK(const Eigen::Affine3d &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal,
                 const moveit_msgs::AttachedCollisionObject &attached_object);

  bool PlanToJointTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                         moveit::planning_interface::MoveGroupInterface::Plan &plan,
                         const moveit_msgs::RobotState &robot_start_state);
  void InitIKRequest(moveit_msgs::GetPositionIK &srv); // use a explicit srv reference
  void InitIKRequest();                                // use directly the class member srv_
  bool ExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan);
  // Collision object related
  void MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose,
                           const std::string &new_reference_frame);
  void MoveCollisionObject(const std::string &obj_id, const Eigen::Affine3d &new_pose,
                           const std::string &new_reference_frame);
  void MoveCollisionObjects(const std::vector<std::string> &obj_ids, const std::vector<geometry_msgs::Pose> &new_poses,
                            const std::vector<std::string> &new_reference_frames);
  void MoveCollisionObjects(const std::vector<std::string> &obj_ids, const std::vector<Eigen::Affine3d> &new_poses,
                            const std::vector<std::string> &new_reference_frames);
  bool AttachCollisionObject(const std::string &obj_id, const geometry_msgs::PoseStamped &grasping_pose);
  bool DetachCollisionObject(const std::string &obj_id, const geometry_msgs::PoseStamped &placement_pose);
  bool OnWorkspace(const Eigen::Affine3d &robot_base_pose, const Eigen::Affine3d &target_pose) const;
  bool OnCircle(const Eigen::Vector3d &origin, const double radius, const Eigen::Vector3d &target) const;

  moveit_msgs::CollisionObject GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                              const geometry_msgs::Pose &new_pose,
                                                              const std::string &new_reference_frame);
  shape_msgs::Mesh MeshMsgFromFile(const std::string &mesh_path);
  void PopulateLocationConnections();
  bool LocationReachable(const Eigen::Affine3d &origin, const Eigen::Affine3d &destination) const;
  bool PlanAndExecuteCloseGripper(moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface);
  bool PlanAndExecuteOpenGripper(moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface);
  bool
  PlanAndExecuteMoveGripperToNamedTarget(moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface,
                                         const std::string &named_target);

  void ComputeHashes(const Eigen::Affine3d &base_pose, const std::vector<Eigen::Affine3d> &object_poses,
                     const std::string &attached_object, std::size_t &state_hash, std::size_t &state_local_hash,
                     std::vector<std::size_t> &on_workspace_objects, std::vector<std::size_t> &features_hashes) const;
  // visualization
  void Publish(const ros::TimerEvent &event);

  // Test only
  void AddTestCollision(); // TEST ONLY

  // Classes
  enum SketchRules { END, PICK_MISPLACED_OBJECT, PICK_OBSTRUCTING_OBJECT, PLACE_OBJECT };
  struct SketchFeatures {
    std::size_t m;
    std::size_t n;
    std::size_t s;
    bool H;
  };

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
    std::vector<Eigen::Affine3d> stable_object_poses_;
  };

  // OBJECTS
  ros::NodeHandle nh_;
  ros::Publisher pub_placements_;
  ros::Publisher pub_locations_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_robot_state_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer display_timer_;
  geometry_msgs::PoseArray display_placements_;
  geometry_msgs::PoseArray display_locations_;
  visualization_msgs::MarkerArray display_objects_;
  moveit_msgs::DisplayRobotState display_robot_state_;
  Eigen::Affine3d robot_pose_;
  std::map<std::string, Object> objects_;
  std::vector<std::string> object_names_;
  std::map<std::string, std::size_t> object_indices;
  std::vector<Eigen::Affine3d> base_locations_;
  std::unordered_map<std::size_t, std::vector<std::size_t>> location_connections_;
  // TODO: Check all the base locations are connected if not, more sampling needed

  double allowed_distance_between_connected_locations_;
  Eigen::Affine3d robot_origin_;
  std::string robot_grasping_link_;
  moveit_msgs::RobotState::_joint_state_type robot_home_joint_config_;

  Eigen::Affine3d gripper_home_wrt_robot_base;

  Eigen::Vector3d robot_workspace_center_translation_;

  double robot_workspace_radius_;
  double gripper_semiamplitude_;

  std::string robot_root_tf_;
  std::string common_reference_;
  std::string ik_service_name_;

  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient ik_service_client_;
  std::vector<std::string> group_joint_names_;
  std::size_t num_group_joints_;
  moveit_msgs::GetPositionIK srv_;
  std::default_random_engine generator_;
  SketchRules active_sketch_rule_;
  SketchFeatures start_state_sketch_features_;
  double blocking_object_distance_threshold_;

  std::unordered_map<std::size_t, std::vector<Action *>> discovered_valid_actions_;

  // Statistics
  std::size_t num_move_base_, num_pick_, num_place_, num_total_ik_calls_, num_successful_ik_calls_,
      num_total_motion_plans_, num_successful_motion_plans_, reused_valid_actions_;
};
