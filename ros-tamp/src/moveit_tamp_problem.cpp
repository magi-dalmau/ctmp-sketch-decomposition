#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <iostream>
#include <map>
#include <moveit_tamp_problem.hpp>
#include <random>
#include <regex>
#include <ros/ros.h>
#include <tinyxml.h>
#include <vector>

// Main class methods
MoveitTampProblem::MoveitTampProblem(const std::string &filename, const std::string &planning_group,
                                     ros::NodeHandle *nodehandle)
    : nh_(*nodehandle), pub_placements_(nh_.advertise<geometry_msgs::PoseArray>("placements", 0, true)),
      pub_locations_(nh_.advertise<geometry_msgs::PoseArray>("locations", 0, true)),
      pub_objects_(nh_.advertise<visualization_msgs::MarkerArray>("objects", 0, true)),
      move_group_interface_(moveit::planning_interface::MoveGroupInterface(planning_group)) {

  ROS_DEBUG("INITIALIZING PROBLEM");
  // initialize actions
  actions_.push_back(new MoveitTampAction(MoveitTampAction::PICK));
  actions_.push_back(new MoveitTampAction(MoveitTampAction::PLACE));
  actions_.push_back(new MoveitTampAction(MoveitTampAction::MOVE_BASE));
  ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  // Initialize moveit related objects
  group_joint_names_ = move_group_interface_.getJointNames();
  num_group_joints_ = group_joint_names_.size();
  robot_root_tf_ = "base_footprint"; // TODO: obtain robot root tf automatically?
  common_reference_ = nh_.param("common_reference", std::string("world"));
  std::cout << "Planning frame is: " << move_group_interface_.getPlanningFrame() << std::endl;
  InitIKRequest();

  // Load world
  LoadWorld(filename);

  ROS_DEBUG("PROBLEM INITIALIZED");
}

MoveitTampProblem::~MoveitTampProblem() { display_timer_.stop(); }

void MoveitTampProblem::LoadWorld(const std::string &filename) {
  std::map<std::string, shape_msgs::Mesh> meshes_map;
  std::vector<moveit_msgs::CollisionObject> vector_collision_objects;
  moveit_msgs::CollisionObject collision_object;
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  collision_object.header.frame_id = common_reference_;
  collision_object.meshes.resize(1);
  geometry_msgs::Pose origin_mesh;
  origin_mesh.orientation.x = 0;
  origin_mesh.orientation.y = 0;
  origin_mesh.orientation.z = 0;
  origin_mesh.orientation.w = 1;
  origin_mesh.position.x = 0;
  origin_mesh.position.y = 0;
  origin_mesh.position.z = 0;
  collision_object.mesh_poses.push_back(origin_mesh);

  TiXmlDocument doc(filename);
  doc.LoadFile();
  if (!doc.LoadFile()) {
    ROS_ERROR("Failed to load file [%s] with error %s.", filename.c_str(), doc.ErrorDesc());
    throw std::runtime_error("Failed to load file [" + filename + "] with error " + doc.ErrorDesc() + ".");
  }

  // Lambda function string ('true' or '1') to bool
  auto string2bool = [](const std::string &str) {
    return boost::algorithm::to_lower_copy(str).compare("true") == 0 || str.compare("1") == 0;
  };

  TiXmlHandle h_doc(&doc);
  for (auto obj = h_doc.FirstChildElement("problem").FirstChildElement("objects").FirstChildElement("obj").ToElement();
       obj; obj = obj->NextSiblingElement("obj"))

  {
    Object object;
    object.name_ = obj->FirstChildElement("name")->GetText();
    object.mesh_ = obj->FirstChildElement("geom")->GetText();
    std::vector<std::string> values;
    boost::split(values, std::string(obj->FirstChildElement("pose")->GetText()), boost::is_any_of(" "));
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 4; ++j)
        object.pose_.matrix()(i, j) = std::stod(values.at(i * 4 + j));
    object.moveable_ = string2bool(obj->FirstChildElement("moveable")->GetText());

    for (auto sssp = obj->FirstChildElement("sssp"); sssp; sssp = sssp->NextSiblingElement("sssp")) {
      const double x_min = std::stod(sssp->FirstChildElement("xmin")->GetText());
      const double x_max = std::stod(sssp->FirstChildElement("xmax")->GetText());
      const double y_min = std::stod(sssp->FirstChildElement("ymin")->GetText());
      const double y_max = std::stod(sssp->FirstChildElement("ymax")->GetText());
      const double z = std::stod(sssp->FirstChildElement("zmin")->GetText());

      object.surfaces_.push_back(
          SupportingSurface(x_min, x_max, y_min, y_max, Eigen::Affine3d(Eigen::Translation3d(0, 0, z))));

      visualization_msgs::Marker display_surface;
      display_surface.header.frame_id = "world";
      display_surface.ns = "surfaces";
      display_surface.id = display_objects_.markers.size();
      display_surface.type = visualization_msgs::Marker::TRIANGLE_LIST;
      display_surface.action = visualization_msgs::Marker::ADD;
      tf::poseEigenToMsg(object.pose_ * Eigen::Translation3d(0, 0, z), display_surface.pose);
      display_surface.scale.x = 1;
      display_surface.scale.y = 1;
      display_surface.scale.z = 1;
      display_surface.color.r = 1;
      display_surface.color.g = 1;
      display_surface.color.b = 0;
      display_surface.color.a = 1;
      geometry_msgs::Point point;
      point.x = x_min;
      point.y = y_min;
      display_surface.points.push_back(point);
      point.x = x_max;
      display_surface.points.push_back(point);
      point.y = y_max;
      display_surface.points.push_back(point);
      display_surface.points.push_back(point);
      point.x = x_min;
      display_surface.points.push_back(point);
      point.y = y_min;
      display_surface.points.push_back(point);
      display_objects_.markers.push_back(display_surface);
    }

    objects_.insert(std::make_pair(object.name_, object));

    visualization_msgs::Marker display_object;
    display_object.header.frame_id = "world";
    display_object.ns = object.moveable_ ? "moveable_objects" : "non-moveable_objects";
    display_object.id = display_objects_.markers.size();
    display_object.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string mesh_name = object.mesh_.substr(0, object.mesh_.find_last_of('.'));
    display_object.mesh_resource =
        "file://" + filename.substr(0, filename.substr(0, filename.find_last_of('/')).find_last_of('/')) + "/meshes/" +
        mesh_name + ".dae";
    // display_object.mesh_use_embedded_materials = true;
    // TODO(magi.dalmau) solve color problem definition in files
    display_object.action = visualization_msgs::Marker::ADD;
    tf::poseEigenToMsg(object.pose_, display_object.pose);
    display_object.scale.x = 1;
    display_object.scale.y = 1;
    display_object.scale.z = 1;
    if (object.name_.find("red") != std::string::npos) {
      display_object.color.r = 1;
      display_object.color.g = 0;
      display_object.color.b = 0;
      display_object.mesh_use_embedded_materials = false;
    } else if (object.name_.find("green") != std::string::npos) {
      display_object.color.r = 0;
      display_object.color.g = 1;
      display_object.color.b = 0;
      display_object.mesh_use_embedded_materials = false;
    } else if (object.name_.find("blue") != std::string::npos) {
      display_object.color.r = 0;
      display_object.color.g = 0;
      display_object.color.b = 1;
      display_object.mesh_use_embedded_materials = false;
    } else {
      display_object.color.r = 0.5;
      display_object.color.g = 0.5;
      display_object.color.b = 0.5;
      display_object.mesh_use_embedded_materials = false;
    }
    display_object.color.a = 1;
    display_objects_.markers.push_back(display_object);



    // moveit collision objects operations

    // Set the particular values of the collision object
    collision_object.id = object.name_;
    collision_object.pose = display_object.pose;

    // Build mesh msg if not done yet
    const auto mesh_it = meshes_map.find(mesh_name);
    if (mesh_it == meshes_map.end()) {
      std::cout << "Loading mesh " << mesh_name << std::endl;
      auto mesh = MeshMsgFromFile(display_object.mesh_resource);
      meshes_map.insert(std::make_pair(mesh_name, mesh));
      std::cout << "mesh lodaded" << std::endl;
      collision_object.meshes.at(0) = mesh; // note that the mesh vector is already initialized in the common operations
    }else{
      collision_object.meshes.at(0) =
          mesh_it->second; // note that the mesh vector is already initialized in the common operations
    } 
    
    std::cout << "Inserted collision mesh" << std::endl;
    // Pushback the collision object
    vector_collision_objects.push_back(collision_object);
  }

  display_placements_.header.frame_id = "world";
  for (const auto &object : objects_) {
    for (const auto &surface : object.second.surfaces_) {
      for (const auto &placement : surface.placements_) {
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(placement, pose);
        display_placements_.poses.push_back(pose);
      }
    }
  }
  // Apply (syncronously) the created collision objects
  planning_scene_interface_.applyCollisionObjects(vector_collision_objects);

  display_timer_ = nh_.createTimer(ros::Duration(0.1), &MoveitTampProblem::Publish, this);
}
shape_msgs::Mesh MoveitTampProblem::MeshMsgFromFile(const std::string &mesh_path) {
  shapes::ShapeMsg mesh_msg;
  shapes::Mesh *m = shapes::createMeshFromResource(mesh_path);
  shapes::constructMsgFromShape(m, mesh_msg);
  delete m;
  return boost::get<shape_msgs::Mesh>(mesh_msg);
}

void MoveitTampProblem::Publish(const ros::TimerEvent &event) {
  pub_placements_.publish(display_placements_);
  pub_locations_.publish(display_locations_);
  pub_objects_.publish(display_objects_);
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

// Supporting Surface Methods
MoveitTampProblem::SupportingSurface::SupportingSurface(double x_min, double x_max, double y_min, double y_max,
                                                        const Eigen::Affine3d &surface_pose)
    : pose_(surface_pose) {
  min_(0) = x_min;
  min_(1) = y_min;
  max_(0) = x_max;
  max_(1) = y_max;
};

double MoveitTampProblem::SupportingSurface::area() const { return (max_ - min_).prod(); }
bool MoveitTampProblem::SupportingSurface::on(const Eigen::Vector3d &position) const {
  const auto v = pose_.inverse() * position;
  const double tol = 1e-6;
  return (v(0) >= min_(0) - tol && v(0) <= max_(0) + tol) && (v(1) >= min_(1) - tol && v(1) <= max_(1) + tol) &&
         (v(2) >= 0. - tol && v(2) <= 0. + tol);
}
void MoveitTampProblem::SupportingSurface::sample() {
  const unsigned int num_bins = std::max(1., std::ceil(sqrt(double(placements_.size()))));
  std::vector<double> weights_x(num_bins, 0), weights_y(num_bins, 0), weights_yaw(num_bins, 0);
  std::vector<double> intervals_x(num_bins + 1), intervals_y(num_bins + 1), intervals_yaw(num_bins + 1);
  double x, y, yaw;
  for (unsigned int i = 0; i <= num_bins; ++i) {
    intervals_x.at(i) = min_(0) + (max_(0) - min_(0)) * double(i) / double(num_bins);
    intervals_y.at(i) = min_(1) + (max_(1) - min_(1)) * double(i) / double(num_bins);
    intervals_yaw.at(i) = -M_PI + 2. * M_PI * double(i) / double(num_bins);
  }
  for (const auto &placement : placements_) {
    const auto local_pose = pose_.inverse() * placement;
    weights_x.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(0) - min_(0)) /
                                                                 (max_(0) - min_(0)) * double(num_bins))))++;
    weights_y.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(1) - min_(1)) /
                                                                 (max_(1) - min_(1)) * double(num_bins))))++;
    weights_yaw.at(
        std::min(num_bins - 1, (unsigned int)std::floor((atan2(local_pose.matrix()(1, 0) - local_pose.matrix()(0, 1),
                                                               local_pose.matrix()(0, 0) + local_pose.matrix()(1, 1)) +
                                                         M_PI) /
                                                        (2. * M_PI) * double(num_bins))))++;
  }

  // for (auto value : weights_x) {
  //   std::cout << ": " << std::string(value, '*') << std::endl;
  // }
  // std::cout << std::endl;
  // for (auto value : weights_y) {
  //   std::cout << ": " << std::string(value, '*') << std::endl;
  // }
  // std::cout << std::endl;
  // for (auto value : weights_yaw) {
  //   std::cout << ": " << std::string(value, '*') << std::endl;
  // }
  // std::cout << std::endl;

  for (unsigned int i = 0; i < num_bins; ++i) {
    weights_x.at(i) = 1. / (1e-6 + weights_x.at(i));
    weights_y.at(i) = 1. / (1e-6 + weights_y.at(i));
    weights_yaw.at(i) = 1. / (1e-6 + weights_yaw.at(i));
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::piecewise_constant_distribution<double> distribution_x(intervals_x.begin(), intervals_x.end(),
                                                              weights_x.begin());
  std::piecewise_constant_distribution<double> distribution_y(intervals_y.begin(), intervals_y.end(),
                                                              weights_y.begin());
  std::piecewise_constant_distribution<double> distribution_yaw(intervals_yaw.begin(), intervals_yaw.end(),
                                                                weights_yaw.begin());
  placements_.push_back(pose_ * Eigen::Translation3d(distribution_x(generator), distribution_y(generator), 0) *
                        Eigen::AngleAxisd(distribution_yaw(generator), Eigen::Vector3d::UnitZ()));
}