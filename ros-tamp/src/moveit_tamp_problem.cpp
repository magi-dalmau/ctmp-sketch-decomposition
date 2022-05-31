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

// MAIN CLASS
// Constructor related methods
MoveitTampProblem::MoveitTampProblem(const std::string &filename, const std::string &planning_group,
                                     ros::NodeHandle *nodehandle)
    : nh_(*nodehandle), pub_placements_(nh_.advertise<geometry_msgs::PoseArray>("placements", 0, true)),
      pub_locations_(nh_.advertise<geometry_msgs::PoseArray>("locations", 0, true)),
      pub_objects_(nh_.advertise<visualization_msgs::MarkerArray>("objects", 0, true)),
      move_group_interface_(moveit::planning_interface::MoveGroupInterface(planning_group)),
      generator_(std::chrono::system_clock::now().time_since_epoch().count()) {

  ROS_DEBUG("INITIALIZING PROBLEM");

  // PARAMS
  ik_service_name_ = nh_.param("ik_service_name", std::string("/compute_ik"));
  common_reference_ = nh_.param("common_reference", std::string("world"));
  robot_home_joint_config_.name = move_group_interface_.getJointNames();
  robot_home_joint_config_.position =
      std::vector<double>{0.332312, 0.587528, -0.981651, -2.68383, 2.18354, -0.634898, 0.107869, -2.04618};
  // for(const auto &value :move_group_interface_.getCurrentJointValues()){
  //   std::cout << value << std::endl;
  // }
  // robot_home_joint_config_.position = ;
  gripper_home_wrt_robot_base.fromPositionOrientationScale(Eigen::Vector3d(0.466, -0.258, 1.101),
                                                           Eigen::Quaterniond(0.214, -0.517, 0.318, 0.766),
                                                           Eigen::Vector3d(1., 1., 1.)); // TODO: set from roslaunch
  robot_workspace_center_translation_ = Eigen::Vector3d(0.093, 0.014, 1.070);            // TODO: set from roslaunch
  robot_workspace_radio_ = 0.6; // TODO_worskpace radio coarse estimation, should be refined and setted form roslaunch
  // Initialize moveit related objects
  group_joint_names_ = move_group_interface_.getJointNames();
  // for (const auto &name : group_joint_names_) {
  //   std::cout << name << std::endl;
  // }
  num_group_joints_ = group_joint_names_.size();
  robot_grasping_link_ = move_group_interface_.getEndEffectorLink();
  robot_root_tf_ = move_group_interface_.getPlanningFrame();

  std::cout << "Planning frame is: " << move_group_interface_.getPlanningFrame()
            << " and end eff link: " << move_group_interface_.getEndEffectorLink() << std::endl;

  // Init IK client
  if (!ros::service::waitForService(ik_service_name_, ros::Duration(10))) {
    throw std::runtime_error("IK service: " + ik_service_name_ + " not found");
  }
  ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>(
      ik_service_name_, true); // Client with persistent connection due to the elevated number of requests
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
  auto string2pose = [](const std::string &str) {
    Eigen::Affine3d pose;
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 4; ++j)
        pose.matrix()(i, j) = std::stod(values.at(i * 4 + j));

    return pose;
  };

  auto string2rot = [](const std::string &str) {
    Eigen::Matrix3d rotation_matrix;
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        rotation_matrix(i, j) = std::stod(values.at(i * 3 + j));

    return rotation_matrix;
  };

  auto string2vector = [](const std::string &str) {
    Eigen::Vector3d vector;
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < 3; ++i)
      vector(i) = std::stod(values.at(i));

    return vector;
  };

  unsigned int num_surfaces = 0;
  TiXmlHandle h_doc(&doc);

  // PARSE ROBOTS
  // TODO:#FUTURE IMPROVEMENT: Only one robot considered, multirobot pending
  auto rob = h_doc.FirstChildElement("problem").FirstChildElement("robots").FirstChildElement("robot").ToElement();
  robot_origin_ = string2pose(rob->FirstChildElement("basepose")->GetText());
  std::cout << "loaded robot origin" << std::endl;
  // PARSE OBJECTS
  for (auto obj = h_doc.FirstChildElement("problem").FirstChildElement("objects").FirstChildElement("obj").ToElement();
       obj; obj = obj->NextSiblingElement("obj"))

  {
    Object object;
    object.name_ = obj->FirstChildElement("name")->GetText();
    object.mesh_ = obj->FirstChildElement("geom")->GetText();
    object.pose_ = string2pose(obj->FirstChildElement("pose")->GetText());
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
      display_surface.header.frame_id = common_reference_;
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

    num_surfaces += object.surfaces_.size();

    // POPULATE GRASPS
    auto grasps = obj->FirstChildElement("grasps");
    if (grasps) {
      for (auto grasp = grasps->FirstChildElement("gc"); grasp; grasp = grasp->NextSiblingElement("gc")) {
        auto pose = string2pose(grasp->FirstChildElement("template")->GetText());
        auto axis = string2vector(grasp->FirstChildElement("axis")->GetText());
        for (unsigned int i = 0; i < 4; ++i)
          object.grasps_.push_back(pose * Eigen::AngleAxisd(M_PI_2 * i, axis));
      }
      for (auto grasp = grasps->FirstChildElement("gf"); grasp; grasp = grasp->NextSiblingElement("gf")) {
        object.grasps_.push_back(string2pose(grasp->GetText()));
      }
    }

    // POPULATE SOPS
    for (auto sop = obj->FirstChildElement("sop"); sop; sop = sop->NextSiblingElement("sop")) {

      auto pose = Eigen::Affine3d().fromPositionOrientationScale(
          Eigen::Vector3d(1, 1, 1), string2rot(sop->FirstChildElement("template")->GetText()),
          Eigen::Vector3d(1, 1, 1));
      auto axis = string2vector(sop->FirstChildElement("axis")->GetText());
      for (unsigned int i = 0; i < 4; ++i)
        object.stable_object_poses_.push_back(pose * Eigen::AngleAxisd(M_PI_2 * i, axis));
    }

    objects_.insert(std::make_pair(object.name_, object));
    object_names_.push_back(object.name_);
    object_indices.insert(std::make_pair(object.name_, object_names_.size() - 1));
    std::cout << "Inserted object " << object.name_ << std::endl;

    visualization_msgs::Marker display_object;
    display_object.header.frame_id = common_reference_;
    display_object.ns = object.moveable_ ? "moveable_objects" : "non-moveable_objects";
    display_object.id = display_objects_.markers.size();
    display_object.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string mesh_name = object.mesh_.substr(0, object.mesh_.find_last_of('.'));
    display_object.mesh_resource =
        "file://" + filename.substr(0, filename.substr(0, filename.find_last_of('/')).find_last_of('/')) + "/meshes/" +
        mesh_name + ".dae";
    // display_object.mesh_use_embedded_materials = true;
    // TODO #FUTURE MINOR:(magi.dalmau) solve color problem definition in files
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
    } else {
      collision_object.meshes.at(0) =
          mesh_it->second; // note that the mesh vector is already initialized in the common operations
    }

    std::cout << "Inserted collision mesh" << std::endl;
    // Pushback the collision object
    vector_collision_objects.push_back(collision_object);
  }

  // SAMPLE AND POPULATE PLACEMENTS

  // TODO: #LAUNCHFILE PARAMS: Setejar per parametres
  // TODO: No muestrear en el abismo
  // TODO: Incluir starting object poses as placements
  // TODO: Placements deberían pertenecer al problem y estar en world coords
  std::size_t num_placements = 100 - 2 * 14;

  std::vector<double> weights(num_surfaces);
  for (unsigned int k = 0; k < num_placements; ++k) {
    unsigned int i = 0;
    for (const auto &object : objects_) {
      for (const auto &surface : object.second.surfaces_) {
        weights.at(i++) = surface.area() / (1e-6 + double(surface.placements_.size()));
      }
    }

    std::discrete_distribution<std::size_t> distribution(weights.begin(), weights.end());
    unsigned int j = distribution(generator_);
    i = 0;
    for (auto &object : objects_) {

      if (i + object.second.surfaces_.size() <= j) {
        i += object.second.surfaces_.size();
      } else {
        object.second.surfaces_.at(j - i).sample();
        break;
      }
    }
  }
  std::cout << "Loaded Placements" << std::endl;

  // Display placements
  display_placements_.header.frame_id = common_reference_;
  for (const auto &object : objects_) {
    for (const auto &surface : object.second.surfaces_) {
      for (const auto &placement : surface.placements_) {
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(object.second.pose_ * placement, pose);
        display_placements_.poses.push_back(pose);
      }
    }
  }
  // Apply (syncronously) the created collision objects
  planning_scene_interface_.applyCollisionObjects(vector_collision_objects);

  // Sample and store possible base locations
  BaseStateSpace base_space;
  // TODO: #LAUNCHFILE PARAMS: carregar tot aixo per parametres
  base_space.x_min = -2.5;
  base_space.y_min = -2.5;
  base_space.x_max = 2.5;
  base_space.y_max = 2.5;
  double dist = 0.5;
  std::size_t locations_amount = 100;

  allowed_distance_between_connected_locations_ = 0;
  std::map<std::string, std::vector<double>> object_to_origin_distances;
  display_locations_.header.frame_id = common_reference_;
  base_locations_.push_back(robot_origin_);
  geometry_msgs::Pose robot_pose;
  tf::poseEigenToMsg(robot_origin_, robot_pose);
  display_locations_.poses.push_back(robot_pose);
  while (base_locations_.size() < locations_amount) {
    for (const auto &object : objects_) {
      if (object.second.surfaces_.empty())
        continue;
      auto iter = object_to_origin_distances.find(object.first);

      if (iter == object_to_origin_distances.end()) {
        std::vector<double> max_limit_vector;
        for (size_t i = 0; i < object.second.surfaces_.size(); i++) {
          max_limit_vector.push_back(std::numeric_limits<double>::max());
        }
        auto iter = (object_to_origin_distances.insert(std::make_pair(object.first, max_limit_vector))).first;
      }
      for (size_t k = 0; k < object.second.surfaces_.size(); k++) {
        const auto &surface = object.second.surfaces_.at(k);
        double min_dist_surface_to_origin = iter->second.at(k);
        std::vector<double> weights(4);
        weights.at(0) = weights.at(2) = surface.max_(0) - surface.min_(0);
        weights.at(1) = weights.at(3) = surface.max_(1) - surface.min_(1);
        std::discrete_distribution<std::size_t> disc_distr(weights.begin(), weights.end());
        std::uniform_real_distribution<double> unif_distr;
        std::normal_distribution<double> normal_distr;
        unsigned int i = disc_distr(generator_);
        double u = unif_distr(generator_);
        double z = normal_distr(generator_);

        double x, y, yaw;
        if (i == 0) {
          x = surface.min_(0) * u + surface.max_(0) * (1 - u);
          y = surface.max_(1) + dist + (0.5 * 0.1) * z;
          yaw = 1.5 * M_PI + (0.5 * 30. / 180. * M_PI) * z;
        } else if (i == 1) {
          x = surface.max_(0) + (dist + (0.5 * 0.1) * z);
          y = surface.min_(1) * u + surface.max_(1) * (1 - u);
          yaw = M_PI + (0.5 * 30. / 180. * M_PI) * z;
        } else if (i == 2) {
          x = surface.min_(0) * u + surface.max_(0) * (1 - u);
          y = surface.min_(1) - (dist + (0.5 * 0.1) * z);
          yaw = 0.5 * M_PI + (0.5 * 30. / 180. * M_PI) * z;
        } else {
          x = surface.min_(0) - (dist + (0.5 * 0.1) * z);
          y = surface.min_(1) * u + surface.max_(1) * (1 - u);
          yaw = 0 + (0.5 * 30. / 180. * M_PI) * z;
        }
        Eigen::Affine3d affine = object.second.pose_ * surface.pose_ * Eigen::Translation3d(x, y, 0) *
                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        x = affine.translation()(0);
        y = affine.translation()(1);
        affine.translation()(2) = 0;
        yaw = Eigen::AngleAxisd(affine.rotation()).angle();
        if (base_space.isValid(x, y, yaw)) {
          base_locations_.push_back(affine);
          geometry_msgs::Pose pose;
          tf::poseEigenToMsg(affine, pose);
          display_locations_.poses.push_back(pose);
          double dist_to_origin = (affine.translation() - robot_origin_.translation()).norm();
          if (dist_to_origin < iter->second.at(k))
            iter->second.at(k) = dist_to_origin;
        }
      }
    }
  }
  std::cout << "Loaded Locations" << std::endl;

  // compute the maximum allowed movement distance of the robot as the maximum of the shortest distances between
  // each surface and the robot origin
  allowed_distance_between_connected_locations_ = 0;
  for (const auto &object_distances : object_to_origin_distances) {
    double max = *std::max_element(object_distances.second.begin(), object_distances.second.end());
    if (max > allowed_distance_between_connected_locations_)
      allowed_distance_between_connected_locations_ = max;
  }
  std::cout << "Computed max travelling distance " << allowed_distance_between_connected_locations_ << std::endl;

  PopulateLocationConnections();

  // Display location coonections
  visualization_msgs::Marker display_location_connections;
  display_location_connections.header.frame_id = common_reference_;
  display_location_connections.ns = "location_connections";
  display_location_connections.id = display_objects_.markers.size();
  display_location_connections.type = visualization_msgs::Marker::LINE_STRIP;
  display_location_connections.action = visualization_msgs::Marker::ADD;
  tf::poseEigenToMsg(Eigen::Affine3d::Identity(), display_location_connections.pose);
  display_location_connections.scale.x = 0.01;
  display_location_connections.scale.y = 1;
  display_location_connections.scale.z = 1;
  display_location_connections.color.r = 1;
  display_location_connections.color.g = 1;
  display_location_connections.color.b = 0;
  display_location_connections.color.a = 1;
  for (std::size_t i = 0; i < base_locations_.size(); ++i) {
    std::size_t hash = 0;
    MoveitTampState::CombineHashPose(hash, base_locations_.at(i));
    auto connections = location_connections_.find(hash);
    if (connections != location_connections_.end()) {
      geometry_msgs::Point start;
      tf::pointEigenToMsg(base_locations_.at(i).translation(), start);
      for (auto j : connections->second) {
        if (j != i) {
          geometry_msgs::Point goal;
          tf::pointEigenToMsg(base_locations_.at(j).translation(), goal);
          display_location_connections.points.push_back(start);
          display_location_connections.points.push_back(goal);
        }
      }
    }
  }
  if (!display_location_connections.points.empty())
    display_objects_.markers.push_back(display_location_connections);

  display_timer_ = nh_.createTimer(ros::Duration(0.1), &MoveitTampProblem::Publish, this);
  std::cout << "World loaded" << std::endl;
}

void MoveitTampProblem::PopulateLocationConnections() {

  location_connections_.reserve(base_locations_.size());
  for (size_t i = 0; i < base_locations_.size(); i++) {
    std::vector<std::size_t> connections;
    for (size_t j = 0; j < base_locations_.size(); j++) {
      if (i == j)
        continue;
      if (LocationReachable(base_locations_.at(i), base_locations_.at(j))) {
        connections.push_back(j);
      }
    }
    std::size_t hash = 0;
    MoveitTampState::CombineHashPose(hash, base_locations_.at(i));
    location_connections_.insert(std::make_pair(hash, connections));
  }
}

bool MoveitTampProblem::LocationReachable(const Eigen::Affine3d &origin, const Eigen::Affine3d &destination) const {
  return Eigen::Vector3d(destination.translation() - origin.translation()).squaredNorm() <=
         allowed_distance_between_connected_locations_ * allowed_distance_between_connected_locations_;
}

// Visualization methods
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

// Motion planning related methods
bool MoveitTampProblem::ComputeIK(const geometry_msgs::Pose &pose_goal,
                                  moveit_msgs::RobotState::_joint_state_type &joint_goal) {

  moveit_msgs::RobotState::_joint_state_type temp_solution;
  // Reuse srv request and change the goal pose
  srv_.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  srv_.request.ik_request.pose_stamped.pose = pose_goal;
  srv_.request.ik_request.robot_state.joint_state.position = move_group_interface_.getRandomJointValues();
  //   std::cout << "Looking for service: " << ik_service_client_.getService() << std::endl;
  //   if (ik_service_client_.exists()) {
  //     std::cout << "Service exists" << std::endl;
  //   } else {
  //     std::cout << "Service not found" << std::endl;
  //   }
  if (!ik_service_client_) {
    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>(
        ik_service_name_, true); // Client with persistent connection due to the elevated number of requests
  }
  if (ik_service_client_.call(srv_)) {
    // std::cout << "Processat el servei de ik"<< std::endl;
    ROS_DEBUG("Error_code: %ld", (long int)srv_.response.error_code.val);
    if (srv_.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_DEBUG("IK Solution found!");
      // TODO: provisiona solution while bug "ik service is returning the whole robot joints instead of only the
      // group joints" is not solved
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

State *const MoveitTampProblem::Start() const {
  std::vector<Eigen::Affine3d> object_poses;
  for (const auto &object_name : object_names_) {
    object_poses.push_back(objects_.at(object_name).pose_);
  }
  return new MoveitTampState(robot_origin_, object_poses);
}

bool MoveitTampProblem::ComputeIK(const Eigen::Affine3d &pose_goal,
                                  moveit_msgs::RobotState::_joint_state_type &joint_goal) {
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose_goal, pose_msg);
  return ComputeIK(pose_msg, joint_goal);
}
void MoveitTampProblem::InitIKRequest(moveit_msgs::GetPositionIK &srv) {
  srv.request.ik_request.group_name = move_group_interface_.getName();
  // std::cout << "Cooking ik request for planning group" << srv.request.ik_request.group_name << std::endl;
  srv.request.ik_request.robot_state.joint_state.name = move_group_interface_.getJointNames();
  srv.request.ik_request.robot_state.joint_state.position = move_group_interface_.getRandomJointValues();
  srv.request.ik_request.avoid_collisions = true;
  srv.request.ik_request.pose_stamped.header.frame_id = "base_footprint";
  srv.request.ik_request.timeout = ros::Duration(1);
  srv.request.ik_request.ik_link_name = "arm_tool_link";
  // std::cout<<"ik request initialized"<<std::endl;
}
void MoveitTampProblem::InitIKRequest() { InitIKRequest(srv_); }

bool MoveitTampProblem::PlanToJoinTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                                         moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                         const moveit_msgs::RobotState &robot_start_state = moveit_msgs::RobotState())

{
  if (robot_start_state.joint_state.position.empty()) {
    move_group_interface_.setStartStateToCurrentState();
  } else {
    move_group_interface_.setStartState(robot_start_state);
  }

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

void MoveitTampProblem::MoveCollisionObjects(
    const std::vector<std::string> &obj_ids, const std::vector<geometry_msgs::Pose> &new_poses,
    const std::vector<std::string> &new_reference_frames = std::vector<std::string>()) {
  assert(obj_ids.size() == new_poses.size());
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.reserve(obj_ids.size());
  if (obj_ids.size() == new_reference_frames.size()) {
    for (size_t i = 0; i < obj_ids.size(); i++) {
      collision_objects.push_back(
          GenerateMoveCollisionObjectMsg(obj_ids.at(i), new_poses.at(i), new_reference_frames.at(i)));
    }
  } else {
    for (size_t i = 0; i < obj_ids.size(); i++) {
      collision_objects.push_back(GenerateMoveCollisionObjectMsg(obj_ids.at(i), new_poses.at(i)));
    }
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);
}

bool MoveitTampProblem::IsGoal(State const *const state) const {
  // Goal = for all X, stick_blueX on table3 and stick_greenX on table4
  // TODO: Goal hardcode only for this PoC
  auto casted_state = dynamic_cast<const MoveitTampState *>(state);
  const auto &table3 = objects_.at("table3");
  const auto &table4 = objects_.at("table4");
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    if (object_names_.at(i).find("stick_blue") != std::string::npos &&
        !table3.surfaces_.front().on(casted_state->GetObjectPoses().at(i).translation())) {
      return false;
    }
    if (object_names_.at(i).find("stick_green") != std::string::npos &&
        !table4.surfaces_.front().on(casted_state->GetObjectPoses().at(i).translation())) {
      return false;
    }
  }

  return true;
}

void MoveitTampProblem::MoveCollisionObjects(
    const std::vector<std::string> &obj_ids, const std::vector<Eigen::Affine3d> &new_poses,
    const std::vector<std::string> &new_reference_frames = std::vector<std::string>()) {
  assert(obj_ids.size() == new_poses.size());
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.reserve(obj_ids.size());
  geometry_msgs::Pose pose;
  if (obj_ids.size() == new_reference_frames.size()) {
    for (size_t i = 0; i < obj_ids.size(); i++) {
      tf::poseEigenToMsg(new_poses.at(i), pose);
      collision_objects.push_back(GenerateMoveCollisionObjectMsg(obj_ids.at(i), pose, new_reference_frames.at(i)));
    }
  } else {
    for (size_t i = 0; i < obj_ids.size(); i++) {
      tf::poseEigenToMsg(new_poses.at(i), pose);
      collision_objects.push_back(GenerateMoveCollisionObjectMsg(obj_ids.at(i), pose));
    }
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void MoveitTampProblem::MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose,
                                            const std::string &new_reference_frame = "") {
  planning_scene_interface_.applyCollisionObject(GenerateMoveCollisionObjectMsg(obj_id, new_pose, new_reference_frame));
}
moveit_msgs::CollisionObject MoveitTampProblem::GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                                               const geometry_msgs::Pose &new_pose,
                                                                               const std::string &new_reference_frame) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = obj_id;
  collision_object.operation = moveit_msgs::CollisionObject::MOVE;
  if (!new_reference_frame.empty()) {
    collision_object.header.frame_id = new_reference_frame;
  }
  collision_object.header.stamp = ros::Time::now();
  collision_object.pose = new_pose;
  return collision_object;
}

// NOT IN USE
bool MoveitTampProblem::AttachCollisionObject(const std::string &obj_id,
                                              const geometry_msgs::PoseStamped &grasping_pose) {

  MoveCollisionObject(obj_id, grasping_pose.pose, grasping_pose.header.frame_id);
  move_group_interface_.attachObject(obj_id);

  return true;
  // TODO: cal verificar que efectivament s'ha completat l'attach? (com al TFG)
}
bool MoveitTampProblem::DetachCollisionObject(const std::string &obj_id,
                                              const geometry_msgs::PoseStamped &placement_pose) {
  move_group_interface_.detachObject(obj_id);
  MoveCollisionObject(obj_id, placement_pose.pose, placement_pose.header.frame_id);
  return true;
  // TODO Verificar que cal canviar el frame reference? un altre cop a world (related amb el todo de attach)
}
bool MoveitTampProblem::OnWorkspace(const Eigen::Affine3d &robot_base_pose, const Eigen::Affine3d &target_pose) const {
  // TODO: Definir centre esfera workspace
  // TODO: Calcular radi esfera workspace

  return OnCircle(robot_base_pose.translation() + robot_workspace_center_translation_, robot_workspace_radio_,
                  target_pose.translation());
}
bool MoveitTampProblem::OnCircle(const Eigen::Vector3d &origin, const double radius,
                                 const Eigen::Vector3d &target) const {

  // NOTE: World coordinates assumed to have z inverse to gravity direction. Assumed that robot can lift its first joint
  // in order to have each maximum reachability distance at the height of the object

  const auto dx = (target(0) > origin(0)) ? (target(0) - origin(0)) : (origin(0) - target(0));
  if (dx > radius)
    return false;
  const auto dy = (target(1) > origin(1)) ? (target(1) - origin(1)) : (origin(1) - target(1));
  if (dy > radius)
    return false;

  return false;
  if (dx + dy <= radius)
    return true;
  return (dx * dx + dy * dy <= radius * radius);
}

std::vector<Action *> MoveitTampProblem::GetValidActions(State const *const state, bool lazy) {

  /*
Cases:
1) Gripper empty:
--possible actions:
---1)Pick(reachable objects)
---2)Move Base (everywhere except current location)
2) Gripper in-use (grasping obj_?ID)
--possible actions:
---1)Place (obj_?ID)
---2)Move Base (connected locations)
*/
  auto casted_state = dynamic_cast<const MoveitTampState *>(state);
  std::vector<Action *> valid_actions;
  const Eigen::Affine3d robot_base_tf_inverse = casted_state->GetRobotBasePose().inverse();

  MoveCollisionObjects(object_names_, casted_state->GetObjectPoses());

  static moveit_msgs::RobotState::_joint_state_type joint_goal;

  // PLACE OR PICK ACTIONS DEPENDING ON IF THE ROBOT HAVE AN ATTACHED OBJECT
  if (casted_state->HasObjectAttached()) {
    const auto attached_object = objects_.find(casted_state->GetAttatchedObject());
    if (attached_object == objects_.end()) {
      ROS_WARN("UNRECOGNIZED TARGET OBJECT IN PLACE ACTION");
    } else {
      move_group_interface_.attachObject(attached_object->first);
      // PLACE ACTIONS
      for (const auto &object : objects_) {
        for (const auto &placement : object.second.placements_) {
          if (!OnWorkspace(casted_state->GetRobotBasePose(), placement))
            continue;

          for (const auto stable_object_pose : attached_object->second.stable_object_poses_) {
            if (!ComputeIK(robot_base_tf_inverse * placement * stable_object_pose * (*casted_state->GetGrasp()),
                           joint_goal))
              continue;
            auto action = new PlaceAction(attached_object->first, joint_goal, placement * stable_object_pose);
            if (!lazy) {
              IsActionValid(state, action);
            }
            valid_actions.push_back(action);
          }
        }
      }
      move_group_interface_.detachObject(attached_object->first);
    }
  } else {
    const auto obj_poses = casted_state->GetObjectPoses();
    for (size_t i = 0; i < obj_poses.size(); i++) {
      if (!OnWorkspace(casted_state->GetRobotBasePose(), obj_poses.at(i)))
        continue;

      const auto object = objects_.find(object_names_.at(i));
      if (object == objects_.end()) {
        ROS_WARN("UNRECOGNIZED TARGET OBJECT IN PICKING ACTION");
        continue;
      }
      MoveCollisionObjects(object_names_, casted_state->GetObjectPoses());

      for (const auto &grasp : object->second.grasps_) {

        if (!ComputeIK(robot_base_tf_inverse * obj_poses.at(i) * grasp, joint_goal))
          continue;

        auto action = new PickAction(object->first, joint_goal, &grasp, obj_poses.at(i));
        if (!lazy) {
          IsActionValid(state, action);
        }

        valid_actions.push_back(action);
      }
    }
  }
  // MOVE BASE ACTIONS
  std::size_t hash = 0;
  casted_state->CombineHashBasePose(hash);
  auto res = location_connections_.find(hash);
  if (res == location_connections_.end()) {
    ROS_ERROR("Unknown current robot base pose");

    return std::vector<Action *>();
  }
  for (const auto &target : res->second) {
    auto action = new MoveBaseAction(base_locations_.at(target));
    valid_actions.push_back(action);
  }

  return valid_actions;
}

bool MoveitTampProblem::IsActionValid(State const *const state, Action *const action, bool lazy) {

  if (auto move_base_action = dynamic_cast<MoveBaseAction *>(action)) {
    return true;
  }

  else if (auto pick_action = dynamic_cast<PickAction *>(action)) {
    if (!lazy) {
      // In addition, checks if non-lazy:
      auto casted_state = dynamic_cast<const MoveitTampState *>(state);
      MoveCollisionObjects(object_names_, casted_state->GetObjectPoses());

      // Valid trajecory from home to (joint) pick pose
      moveit::planning_interface::MoveGroupInterface::Plan to_object_plan;
      if (!PlanToJoinTarget(pick_action->GetJointGoal(), to_object_plan)) {
        return false;
      }

      moveit_msgs::RobotState start_state;
      moveit_msgs::AttachedCollisionObject attached_object;
      geometry_msgs::Pose obj_pose;
      tf::poseEigenToMsg(pick_action->GetTargetObjectPose(), obj_pose);
      attached_object.object = GenerateMoveCollisionObjectMsg(pick_action->GetTargetObjectId(), obj_pose);
      attached_object.link_name = robot_grasping_link_;
      start_state.attached_collision_objects.push_back(attached_object);
      start_state.joint_state = pick_action->GetJointGoal();
      moveit::planning_interface::MoveGroupInterface::Plan to_home_plan;
      if (!PlanToJoinTarget(robot_home_joint_config_, to_home_plan)) {
        return false;
      }

      // move_group_interface_.detachObject(pick_action->GetTargetObjectId());

      // Valid trajectory from pick pose to home
    }

  } else if (auto place_action = dynamic_cast<PlaceAction *>(action)) {

    if (!lazy) {
      // In addition, checks if non-lazy:
      auto casted_state = dynamic_cast<const MoveitTampState *>(state);
      MoveCollisionObjects(object_names_, casted_state->GetObjectPoses());
      move_group_interface_.attachObject(place_action->GetTargetObjectId());

      // Valid trajecory from home to (joint) pick pose
      moveit::planning_interface::MoveGroupInterface::Plan to_object_plan;
      if (!PlanToJoinTarget(place_action->GetJointGoal(), to_object_plan)) {
        return false;
      }
      move_group_interface_.detachObject(place_action->GetTargetObjectId());

      moveit_msgs::RobotState start_state;
      start_state.joint_state = place_action->GetJointGoal();

      moveit::planning_interface::MoveGroupInterface::Plan to_home_plan;
      if (!PlanToJoinTarget(robot_home_joint_config_, to_home_plan, start_state)) {
        return false;
      }

      // Valid trajectory from pick pose to home
    }
  }
  return true;
}

State *const MoveitTampProblem::GetSuccessor(State const *const state, Action const *const action) {
  auto casted_state = dynamic_cast<const MoveitTampState *>(state);
  // TODO: Maybe a more elegant way to organize kinds of actions, maybe each action should have an Apply method but then
  // is state dependent...
  if (auto move_base_action = dynamic_cast<const MoveBaseAction *>(action)) {
    if (casted_state->HasObjectAttached()) {
      std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
      new_object_poses.at(object_indices.at(casted_state->GetAttatchedObject())) =
          move_base_action->GetTargetLocation() * casted_state->GetRobotBasePose().inverse() *
          new_object_poses.at(object_indices.at(casted_state->GetAttatchedObject()));
      return new MoveitTampState(move_base_action->GetTargetLocation(), new_object_poses,
                                 casted_state->GetAttatchedObject());

    } else {
      return new MoveitTampState(move_base_action->GetTargetLocation(), casted_state->GetObjectPoses(),
                                 casted_state->GetAttatchedObject());
    }

  } else if (auto pick_action = dynamic_cast<const PickAction *>(action)) {

    std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
    new_object_poses.at(object_indices.at(pick_action->GetTargetObjectId())) =
        casted_state->GetRobotBasePose() * gripper_home_wrt_robot_base * (*pick_action->GetGrasp());
    return new MoveitTampState(casted_state->GetRobotBasePose(), new_object_poses, pick_action->GetTargetObjectId());

  } else if (auto place_action = dynamic_cast<const PlaceAction *>(action)) {
    std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
    new_object_poses.at(object_indices.at(place_action->GetTargetObjectId())) = place_action->GetTargetObjectPose();
    return new MoveitTampState(casted_state->GetRobotBasePose(), new_object_poses);
  } else {
    ROS_ERROR("UNRECOGNIZED ACTION CLASS");
    return nullptr;
  }
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

// FRIEND CLASSES
//  Supporting Surface Methods
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