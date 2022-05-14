#include <moveit_tamp_problem.hpp>

MoveitTampProblem::MoveitTampProblem(const std::string &planning_group, ros::NodeHandle *nodehandle)
    : nh_(*nodehandle), move_group_interface_(moveit::planning_interface::MoveGroupInterface(planning_group)) {
  std::cout << "INITIALIZING PROBLEM" << std::endl;
  actions_.push_back(new MoveitTampAction(MoveitTampAction::PICK));
  actions_.push_back(new MoveitTampAction(MoveitTampAction::PLACE));
  actions_.push_back(new MoveitTampAction(MoveitTampAction::MOVE_BASE));
  ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

  group_joint_names_ = move_group_interface_.getJointNames();
  num_group_joints_ = group_joint_names_.size();
  robot_root_tf_ = "base_footprint";
  std::cout << "Planning frame is: " << move_group_interface_.getPlanningFrame() << std::endl;
  InitIKRequest();
  std::cout << "FINSIH CONSTRUCTOR" << std::endl;
}

bool MoveitTampProblem::ComputeIK(const geometry_msgs::Pose &pose_goal,
                                  moveit_msgs::RobotState::_joint_state_type &joint_goal) {

  moveit_msgs::RobotState::_joint_state_type temp_solution;
  // TODO: potser en el constructor fer un wait for service (ik)
  // Reuse srv request and change the goal pose
  srv_.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  srv_.request.ik_request.pose_stamped.pose = pose_goal;
  //   std::cout << "Looking for service: " << ik_service_client_.getService() << std::endl;
  //   if (ik_service_client_.exists()) {
  //     std::cout << "Service exists" << std::endl;
  //   } else {
  //     std::cout << "Service not found" << std::endl;
  //   }
  if (ik_service_client_.call(srv_)) {
    // std::cout << "Processat el servei de ik"<< std::endl;
    ROS_DEBUG("Error_code: %ld", (long int)srv_.response.error_code.val);
    if (srv_.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_DEBUG("IK Solution found!");
      // TODO: provisiona solution while bug "ik service is returning the whole robot joints instead of only the group
      // joints" is not solved
      temp_solution = srv_.response.solution.joint_state;
      joint_goal.name.resize(num_group_joints_);
      joint_goal.position.resize(num_group_joints_);
      std::map<std::string, std::size_t> solution_names_map;
      for (std::size_t i = 0; i < temp_solution.name.size(); i++) {
        // std::string name = temp_solution.name.at(i);
        solution_names_map.insert(std::make_pair(temp_solution.name.at(i), i));
      }
      for (size_t i = 0; i < num_group_joints_; i++) {
        const auto joint_name = group_joint_names_.at(i);
        std::size_t index;
        try {
          index = solution_names_map.at(joint_name);
        } catch (std::out_of_range) {
          ROS_ERROR_STREAM("wrong joint " << joint_name << "it is not present in temp solution");
          return false;
        }
        joint_goal.name.at(i) = joint_name;
        joint_goal.position.at(i) = temp_solution.position.at(index);
      }
      return true;
    } else {
      ROS_ERROR("Solution not found");
      return false;
    }
  } else {
    std::cout << "No s'ha processat el servei de ik" << std::endl;
    ROS_ERROR_STREAM("Failed to call service " << ik_service_client_.getService());
    return false;
  }
}

void MoveitTampProblem::InitIKRequest(moveit_msgs::GetPositionIK &srv) {
  srv.request.ik_request.group_name = move_group_interface_.getName();
  // std::cout << "Cooking ik request for planning group" << srv.request.ik_request.group_name << std::endl;
  srv.request.ik_request.robot_state.joint_state.name = move_group_interface_.getJointNames();
  srv.request.ik_request.robot_state.joint_state.position =
      move_group_interface_.getRandomJointValues(); // TODO: caldria anar canviant?
  srv.request.ik_request.avoid_collisions = true;
  srv.request.ik_request.pose_stamped.header.frame_id = "base_footprint";
  srv.request.ik_request.timeout = ros::Duration(1);
  srv.request.ik_request.ik_link_name = "arm_tool_link";
}
void MoveitTampProblem::InitIKRequest() { InitIKRequest(srv_); }

bool MoveitTampProblem::PlanToJoinTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                                         moveit::planning_interface::MoveGroupInterface::Plan &plan)

{
  move_group_interface_.setStartStateToCurrentState();
  move_group_interface_.setJointValueTarget(joint_goal);
  if (!move_group_interface_.plan(plan)) {
    ROS_ERROR("Planning failed");
    return false;
  } else {

    ROS_DEBUG_STREAM("Plan found in " << plan.planning_time_ << " seconds");
    return true;
  }
}

bool MoveitTampProblem::ExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
  ROS_DEBUG_STREAM("Plan will be executed now");
  auto result = move_group_interface_.execute(plan);
  ROS_DEBUG_STREAM("Plan returned " << result);
  return (result == result.SUCCESS);
}

void MoveitTampProblem::MoveCollisionObjects(const std::vector<std::string> &obj_ids,
                                             const std::vector<geometry_msgs::Pose> &new_poses) {
  assert(obj_ids.size() == new_poses.size());
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.reserve(obj_ids.size());
  for (size_t i = 0; i < obj_ids.size(); i++) {
    collision_objects.push_back(GenerateMoveCollisionObjectMsg(obj_ids.at(i), new_poses.at(i)));
  }
  planning_scene_interface_.applyCollisionObjects(collision_objects);
}
void MoveitTampProblem::MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose) {
  planning_scene_interface_.applyCollisionObject(GenerateMoveCollisionObjectMsg(obj_id, new_pose));
}
moveit_msgs::CollisionObject MoveitTampProblem::GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                                               const geometry_msgs::Pose &new_pose) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = obj_id;
  collision_object.operation = moveit_msgs::CollisionObject::MOVE;
  collision_object.header.frame_id = robot_root_tf_;
  collision_object.header.stamp = ros::Time::now();
  collision_object.pose = new_pose;
  return collision_object;
}

// TEST ONLY
void MoveitTampProblem::AddTestCollision() {
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = "box1";
  collision_object.operation = collision_object.ADD;
  collision_object.header.frame_id = "base_footprint";
  collision_object.pose.orientation.x = 0;
  collision_object.pose.orientation.y = 0;
  collision_object.pose.orientation.z = 0;
  collision_object.pose.orientation.w = 1.0;
  collision_object.pose.position.x = 0.5;
  collision_object.pose.position.y = 0.5;
  collision_object.pose.position.z = 1;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  primitive_pose.position.x = 0;
  primitive_pose.position.y = 0;
  primitive_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(primitive_pose);

  planning_scene_interface_.applyCollisionObject(collision_object);
}
