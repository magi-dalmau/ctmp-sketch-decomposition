#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <iostream>
#include <map>
#include <moveit/robot_state/conversions.h>
#include <moveit_tamp_problem.hpp>
#include <random>
#include <regex>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tinyxml.h>
#include <unordered_set>
#include <vector>

// template <class T> inline void hash_combine(std::size_t &s, const T &v) {
//   std::hash<T> h;
//   s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
// };

// void CombineHashPose(std::size_t &hash, const Eigen::Affine3d &pose) {
//   const auto matrix = 1000.0 * pose.matrix();
//   for (unsigned int i = 0; i < 3; ++i) {
//     for (unsigned int j = i; j < 4; ++j) {
//       hash_combine<double>(hash, trunc(matrix(i, j)));
//     }
//   }
// }

// void CombineHashTranslation(std::size_t &hash, const Eigen::Vector3d &translation) {
//   for (size_t i = 0; i < 3; i++) {
//     hash_combine<double>(hash, trunc(translation(i)));
//   }
// }

// void CombineHashRotation(std::size_t &hash, const Eigen::Matrix3d &matrix) {
//   for (unsigned int i = 0; i < 3; ++i) {
//     for (unsigned int j = i; j < 3; ++j) {
//       hash_combine<double>(hash, trunc(matrix(i, j)));
//     }
//   }
// }

// template <class T> inline void CombineHashVector(std::size_t &s, const std::vector<T> &vector) {
//   for (const auto v : vector) {
//     hash_combine<T>(s, v);
//   }
// };

// MAIN CLASS
// Constructor related methods
MoveitTampProblem::MoveitTampProblem(const std::string &filename, const std::string &planning_group,
                                     ros::NodeHandle *nodehandle)
    : nh_(*nodehandle), pub_placements_(nh_.advertise<geometry_msgs::PoseArray>("placements", 0, true)),
      pub_locations_(nh_.advertise<geometry_msgs::PoseArray>("locations", 0, true)),
      pub_objects_(nh_.advertise<visualization_msgs::MarkerArray>("objects", 0, true)),
      pub_robot_state_(nh_.advertise<moveit_msgs::DisplayRobotState>("robot_state", 0, true)),
      move_group_interface_(moveit::planning_interface::MoveGroupInterface(planning_group)),
      generator_(std::chrono::system_clock::now().time_since_epoch().count()) {

  ROS_DEBUG("INITIALIZING PROBLEM");
  // RESET STATISTICS
  num_move_base_ = 0;
  num_pick_ = 0;
  num_place_ = 0;
  num_total_ik_calls_ = 0;
  num_successful_ik_calls_ = 0;
  reused_valid_actions_ = 0;
  num_total_motion_plans_ = 0;
  num_successful_motion_plans_ = 0;
  // PARAMS
  reuse_expansions_ = nh_.param("reuse_expansions", false);
  blocking_object_distance_threshold_ = nh_.param("blocking_object_distance_threshold", 0.2);
  goal_tolerance_radius_ = nh_.param("goal_tolerance_radius_", 0.005);
  ik_service_name_ = nh_.param("ik_service_name", std::string("/compute_ik"));
  common_reference_ = nh_.param("common_reference", std::string("world"));
  discretization_ = nh_.param("discretization_size", 0.05);
  padding_ = nh_.param("sampling_padding", 0.025);
  exclude_fronted_locations_ = nh_.param("exclude_fronted_locations", false);
  block_misplaced_in_goal_ = nh_.param("block_misplaced_in_goal", false);
  goal_region_ = nh_.param("goal_region", true);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(), ros::Time::now(), common_reference_,
                                                     move_group_interface_.getPlanningFrame()));
  robot_home_joint_config_.name = move_group_interface_.getJointNames();
  // robot_home_joint_config_.position =
  //     std::vector<double>{0.349908365757701,  0.08886820063711626, 0.035885547316525906, -2.799176661809888,
  //                         1.5379210924065916, 0.8614694762778718,  -0.8948474111586892,  -2.010910761039485};
  robot_home_joint_config_.position =
      std::vector<double>{0.349908365757701,  0.08886820063711626, 0.035885547316525906, -2.799176661809888,
                          1.5379210924065916, 0.8614694762778718,  -0.8948474111586892,  1.0764951904142515};
  // Home position definied in tiago_pal-gripper_schunk-ft.srdf and set
  // as initial in fake_controllers_pal-gripper_schunk-ft.yaml  (also gripper set to open in the same files)

  // gripper_home_wrt_robot_base.fromPositionOrientationScale(Eigen::Vector3d(0.295, -0.398, 1.400),
  //                                                          Eigen::Quaterniond(0.906, 0.010, -0.226, 0.359),
  //                                                          Eigen::Vector3d(1., 1., 1.)); // TODO: set from roslaunch

  gripper_home_wrt_robot_base.fromPositionOrientationScale(Eigen::Vector3d(0.295, -0.398, 1.400),
                                                           Eigen::Quaterniond(0.015, 0.905, 0.353, 0.236),
                                                           Eigen::Vector3d(1., 1., 1.)); // TODO: set from roslaunch
  robot_workspace_center_translation_ = Eigen::Vector3d(0.093, 0.014, 1.070);            // TODO: set from roslaunch
  robot_workspace_max_radius_ = nh_.param("robot_workspace_max_radius", 0.795);
  0.795; // TODO_worskpace radio coarse estimation, should be refined
  robot_workspace_min_radius_ = nh_.param("robot_workspace_min_radius", 0.395);
  // TODO_worskpace radio coarse estimation, should be refined

  gripper_semiamplitude_ = nh_.param("gripper_semiamplitude", 0.045);

  placements_per_table_adaptative_sampling_ = nh_.param("placements_per_table_adaptative_sampling_", 3);

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
  // TODO publihsing rate by param
  display_timer_ = nh_.createTimer(ros::Duration(0.02), &MoveitTampProblem::Publish, this);

  // TODO set by parameters
  move_group_interface_.setMaxVelocityScalingFactor(1.);
  move_group_interface_.setMaxAccelerationScalingFactor(1.);
  // LOAD WORLD PASSAT ALS FILLS
  //  ROS_DEBUG("PROBLEM INITIALIZED");
}

MoveitTampProblem::~MoveitTampProblem() {
  display_timer_.stop();

  for (auto &state_actions : discovered_valid_actions_) {
    for (auto &action : state_actions.second) {
      delete action;
    }
  }
}

void MoveitTampProblem::LoadWorld(const std::string &filename) {

  double x_distance_arm_tool_link_to_grasping_point = nh_.param("x_dist_arm_tool_link_to_grasping_point", 0.20);

  std::size_t num_grasps = nh_.param("num_grasps", 4);
  std::size_t num_sops = nh_.param("num_sops", 1);

  Eigen::Affine3d arm_tool_link_to_grasp_point_inv =
      (Eigen::Affine3d().fromPositionOrientationScale(Eigen::Vector3d(x_distance_arm_tool_link_to_grasping_point, 0, 0),
                                                      Eigen::Quaterniond(0.7071068, 0.7071068, 0, 0),
                                                      Eigen::Vector3d(1, 1, 1)))
          .inverse();

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
  robot_pose_ = robot_origin_;
  std::cout << "loaded robot origin" << std::endl;
  // PARSE OBJECTS
  for (auto obj = h_doc.FirstChildElement("problem").FirstChildElement("objects").FirstChildElement("obj").ToElement();
       obj; obj = obj->NextSiblingElement("obj"))

  {
    Object object;
    object.name_ = obj->FirstChildElement("name")->GetText();
    object.pose_ = string2pose(obj->FirstChildElement("pose")->GetText());
    std::cout << "Checking name: " << object.name_ << std::endl;
    if (object.name_.find("target") != std::string::npos) {
      goal_positions_.insert(std::make_pair(object.name_, object.pose_ * Eigen::Vector3d(0, 0, 0.001)));

      visualization_msgs::Marker display_object;
      display_object.header.frame_id = common_reference_;
      display_object.ns = "static_objects";
      display_object.id = display_objects_.markers.size();
      display_object.type = visualization_msgs::Marker::CYLINDER;
      display_object.scale.x = 0.045;
      display_object.scale.y = 0.045;
      display_object.scale.z = 0.0001;

      // TODO #FUTURE MINOR:(magi.dalmau) solve color problem definition in files
      display_object.action = visualization_msgs::Marker::ADD;
      tf::poseEigenToMsg(object.pose_, display_object.pose);
      display_object.frame_locked = true;

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

      continue;
    }
    object.mesh_ = obj->FirstChildElement("geom")->GetText();
    object.moveable_ = string2bool(obj->FirstChildElement("moveable")->GetText());
    double surface_offset = 0;
    if (object.moveable_) {
      object.pose_ = object.pose_ * Eigen::Translation3d(0, 0, 0.001);
    } else {
      surface_offset = 0.001;
    }

    for (auto sssp = obj->FirstChildElement("sssp"); sssp; sssp = sssp->NextSiblingElement("sssp")) {
      const double x_min = std::stod(sssp->FirstChildElement("xmin")->GetText());
      const double x_max = std::stod(sssp->FirstChildElement("xmax")->GetText());
      const double y_min = std::stod(sssp->FirstChildElement("ymin")->GetText());
      const double y_max = std::stod(sssp->FirstChildElement("ymax")->GetText());
      const double z = std::stod(sssp->FirstChildElement("zmin")->GetText());
      if (object.surfaces_.empty()) {
        supporting_object_names.push_back(object.name_);
      }
      object.surfaces_.push_back(SupportingSurface(x_min, x_max, y_min, y_max,
                                                   Eigen::Affine3d(Eigen::Translation3d(0, 0, z + surface_offset)),
                                                   discretization_, padding_));

      visualization_msgs::Marker display_surface;
      display_surface.header.frame_id = common_reference_;
      display_surface.ns = "surfaces";
      display_surface.id = display_objects_.markers.size();
      display_surface.type = visualization_msgs::Marker::TRIANGLE_LIST;
      display_surface.action = visualization_msgs::Marker::ADD;
      tf::poseEigenToMsg(object.pose_ * Eigen::Translation3d(0, 0, z), display_surface.pose);
      display_surface.frame_locked = true;
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
        for (unsigned int i = 0; i < num_grasps; ++i)
          // TODO check rot*pos o pos*rot
          object.grasps_.push_back((pose * Eigen::AngleAxisd((2.0 * M_PI / double(num_sops)) * i, axis)) *
                                   arm_tool_link_to_grasp_point_inv);
      }
      for (auto grasp = grasps->FirstChildElement("gf"); grasp; grasp = grasp->NextSiblingElement("gf")) {
        object.grasps_.push_back(string2pose(grasp->GetText()) * arm_tool_link_to_grasp_point_inv);
      }
    }

    // POPULATE SOPS
    for (auto sop = obj->FirstChildElement("sop"); sop; sop = sop->NextSiblingElement("sop")) {
      auto pose = Eigen::Affine3d().fromPositionOrientationScale(
          Eigen::Vector3d::Zero(), string2rot(sop->FirstChildElement("template")->GetText()), Eigen::Vector3d::Ones());
      auto axis = string2vector(sop->FirstChildElement("axis")->GetText());
      for (unsigned int i = 0; i < num_sops; ++i)
        // TODO check rot*pos o pos*rot
        object.stable_object_poses_.push_back(pose * Eigen::AngleAxisd((2.0 * M_PI / double(num_sops)) * i, axis));
    }

    // POPULATE PLACEMENTS
    if (obj->FirstChildElement("attachments")) {
      for (auto attachment = obj->FirstChildElement("attachments")->FirstChildElement("name"); attachment;
           attachment = attachment->NextSiblingElement("nxame")) {
        auto &other_object = objects_.at(attachment->GetText());
        for (auto &surface : object.surfaces_) {
          if (surface.on(object.pose_.inverse() * other_object.pose_.translation()))
            surface.placements_.push_back(object.pose_.inverse() * other_object.pose_);
        }
      }
    }

    objects_.insert(std::make_pair(object.name_, object));
    object_names_.push_back(object.name_);
    object_indices.insert(std::make_pair(object.name_, object_names_.size() - 1));
    std::cout << "Inserted object " << object.name_ << std::endl;
    object_idx_to_marker_idx_.push_back(display_objects_.markers.size());
    visualization_msgs::Marker display_object;
    display_object.header.frame_id = common_reference_;
    display_object.ns = object.moveable_ ? "moveable_objects" : "static_objects";
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
    display_object.frame_locked = true;
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
  LoadPlacements(num_surfaces);

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

  LoadLocations();
  std::cout << "Loaded Locations" << std::endl;

  // compute the maximum allowed movement distance of the robot as the maximum of the shortest distances between
  // each surface and the robot origin

  PopulateLocationConnections();

  // Display location coonections
  visualization_msgs::Marker display_location_connections;
  display_location_connections.header.frame_id = common_reference_;
  display_location_connections.ns = "location_connections";
  display_location_connections.id = display_objects_.markers.size();
  display_location_connections.type = visualization_msgs::Marker::LINE_LIST;
  display_location_connections.action = visualization_msgs::Marker::ADD;
  tf::poseEigenToMsg(Eigen::Affine3d::Identity(), display_location_connections.pose);
  display_location_connections.frame_locked = true;
  display_location_connections.scale.x = 0.01;
  display_location_connections.scale.y = 1;
  display_location_connections.scale.z = 1;
  display_location_connections.color.r = 1;
  display_location_connections.color.g = 1;
  display_location_connections.color.b = 0;
  display_location_connections.color.a = 1;
  for (std::size_t i = 0; i < base_locations_.size(); ++i) {
    std::size_t hash = 0;
    hasher_.CombineHashPose(hash, base_locations_.at(i));
    auto connections = location_connections_.find(hash);
    if (connections != location_connections_.end()) {
      geometry_msgs::Point start;
      tf::pointEigenToMsg(base_locations_.at(i).translation(), start);
      for (auto j : connections->second) {
        if (j > i) {
          geometry_msgs::Point goal;
          tf::pointEigenToMsg(base_locations_.at(j).translation(), goal);
          display_location_connections.points.push_back(start);
          display_location_connections.points.push_back(goal);
          // std::cout << "Distance " << i << "-" << j << " " << hypot(goal.x-start.x,goal.y-start.y) << std::endl;
        }
      }
    }
  }
  if (!display_location_connections.points.empty())
    display_objects_.markers.push_back(display_location_connections);
  display_objects_start_ = display_objects_;
  // POPULATE ROBOT STATE
  display_robot_state_.state.joint_state = robot_home_joint_config_;

  std::cout << "World loaded" << std::endl;
}

void MoveitTampProblem::AddCurrentObjectsToPlacements(const std::vector<Eigen::Affine3d> &object_poses) {
  for (const auto &pose : object_poses) {
    for (const auto &name : supporting_object_names) {
      auto supporting_obj = objects_.find(name);
      if (supporting_obj != objects_.end()) {
        bool found = false;
        for (auto &surface : supporting_obj->second.surfaces_) {
          if (surface.on(supporting_obj->second.pose_.inverse() * pose.translation())) {
            surface.placements_.push_back(supporting_obj->second.pose_.inverse() * pose);
            found = true;
          }
        }
        if (found) {
          break;
        }
      }
    }
  }
}

// void MoveitTampProblem::LoadLocations() {
//   // Sample and store possible base locations
//   BaseStateSpace base_space;
//   base_space.x_min = nh_.param("location_space_x_min", -2.2);
//   base_space.y_min = nh_.param("location_space_y_min", -2.2);
//   base_space.x_max = nh_.param("location_space_x_max", 2.2);
//   base_space.y_max = nh_.param("location_space_y_max", 2.2);
//   double dist = nh_.param("average_sampling_distance_base_table", 0.3);
//   std::size_t locations_amount = nh_.param("locations_amount", 100);

//   std::map<std::string, std::vector<double>> object_to_origin_distances;
//   display_locations_.header.frame_id = common_reference_;
//   base_locations_.push_back(robot_origin_);
//   geometry_msgs::Pose robot_pose;
//   tf::poseEigenToMsg(robot_origin_, robot_pose);
//   display_locations_.poses.push_back(robot_pose);
//   while (base_locations_.size() < locations_amount) {

//     for (const auto &object : objects_) {
//       if (object.second.surfaces_.empty())
//         continue;
//       auto iter = object_to_origin_distances.find(object.first);

//       if (iter == object_to_origin_distances.end()) {
//         std::vector<double> max_limit_vector(object.second.surfaces_.size(), std::numeric_limits<double>::max());

//         // std::cout << "max_lim_vector size: " << max_limit_vector.size() << std::endl;
//         iter = (object_to_origin_distances.insert(std::make_pair(object.first, max_limit_vector))).first;
//       }
//       for (size_t k = 0; k < object.second.surfaces_.size(); k++) {
//         const auto &surface = object.second.surfaces_.at(k);
//         // std::cout << "num surfaces : " << object.second.surfaces_.size() << " distances : " <<
//         iter->second.size()
//         <<
//         // std::endl;
//         double min_dist_surface_to_origin = iter->second.at(k);
//         std::vector<double> weights(4);
//         weights.at(0) = weights.at(2) = surface.max_(0) - surface.min_(0);
//         weights.at(1) = weights.at(3) = surface.max_(1) - surface.min_(1);

//         std::discrete_distribution<std::size_t> disc_distr(weights.begin(), weights.end());
//         std::uniform_real_distribution<double> unif_distr;
//         std::normal_distribution<double> normal_distr;
//         unsigned int i = disc_distr(generator_);
//         double u = unif_distr(generator_);
//         double z = normal_distr(generator_);

//         double x, y, yaw;
//         if (i == 0) {
//           x = surface.min_(0) * u + surface.max_(0) * (1 - u);
//           y = surface.max_(1) + dist + (0.5 * 0.1) * z;
//           yaw = 1.5 * M_PI + (0.5 * 30. / 180. * M_PI) * z;
//         } else if (i == 1) {
//           x = surface.max_(0) + (dist + (0.5 * 0.1) * z);
//           y = surface.min_(1) * u + surface.max_(1) * (1 - u);
//           yaw = M_PI + (0.5 * 30. / 180. * M_PI) * z;
//         } else if (i == 2) {
//           x = surface.min_(0) * u + surface.max_(0) * (1 - u);
//           y = surface.min_(1) - (dist + (0.5 * 0.1) * z);
//           yaw = 0.5 * M_PI + (0.5 * 30. / 180. * M_PI) * z;
//         } else {
//           x = surface.min_(0) - (dist + (0.5 * 0.1) * z);
//           y = surface.min_(1) * u + surface.max_(1) * (1 - u);
//           yaw = 0 + (0.5 * 30. / 180. * M_PI) * z;
//         }

//         Eigen::Affine3d affine = object.second.pose_ * surface.pose_ * Eigen::Translation3d(x, y, 0) *
//                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
//         x = affine.translation()(0);
//         y = affine.translation()(1);
//         affine.translation()(2) = 0;
//         yaw = Eigen::AngleAxisd(affine.rotation()).angle();
//         if (base_space.isValid(x, y, yaw)) {
//           base_locations_.push_back(affine);
//           geometry_msgs::Pose pose;
//           tf::poseEigenToMsg(affine, pose);
//           display_locations_.poses.push_back(pose);
//           double dist_to_origin = (affine.translation() - robot_origin_.translation()).norm();
//           if (dist_to_origin < iter->second.at(k))
//             iter->second.at(k) = dist_to_origin;
//         }
//       }
//     }
//   }
//
// allowed_distance_between_connected_locations_ = 0;
// for (const auto &object_distances : object_to_origin_distances) {
//   double max = *std::max_element(object_distances.second.begin(), object_distances.second.end());
//   if (max > allowed_distance_between_connected_locations_)
//     allowed_distance_between_connected_locations_ = max;
// }
// allowed_distance_between_connected_locations_ += 1e-3;
// std::cout << "Computed max travelling distance " << allowed_distance_between_connected_locations_ << std::endl;
// }

void MoveitTampProblem::LoadLocations() {
  // Sample and store possible base locations
  BaseStateSpace base_space;
  base_space.x_min = nh_.param("location_space_x_min", -2.2);
  base_space.y_min = nh_.param("location_space_y_min", -2.2);
  base_space.x_max = nh_.param("location_space_x_max", 2.2);
  base_space.y_max = nh_.param("location_space_y_max", 2.2);

  std::map<std::string, std::vector<double>> object_to_origin_distances;
  display_locations_.header.frame_id = common_reference_;
  base_locations_.push_back(robot_origin_);
  geometry_msgs::Pose robot_pose;
  tf::poseEigenToMsg(robot_origin_, robot_pose);
  display_locations_.poses.push_back(robot_pose);
  for (const auto &object : objects_) {
    if (object.second.surfaces_.empty())
      continue;
    auto iter = object_to_origin_distances.find(object.first);

    if (iter == object_to_origin_distances.end()) {
      std::vector<double> max_limit_vector(object.second.surfaces_.size(), std::numeric_limits<double>::max());

      // std::cout << "max_lim_vector size: " << max_limit_vector.size() << std::endl;
      iter = (object_to_origin_distances.insert(std::make_pair(object.first, max_limit_vector))).first;
    }

    for (size_t k = 0; k < object.second.surfaces_.size(); k++) {
      const auto &surface = object.second.surfaces_.at(k);
      // TODO Solve issue with base locations
      std::vector<Eigen::Affine3d> local_poses;
      // local_poses.push_back(Eigen::Translation3d(0.5 * (surface.min_(0) + surface.max_(0)),
      //                                            surface.max_(1) + 0.1 + robot_workspace_min_radius_, 0) *
      //                       Eigen::AngleAxisd(1.5 * M_PI, Eigen::Vector3d::UnitZ()));
      // local_poses.push_back(
      //     Eigen::Translation3d(0.5 * (surface.min_(0) + surface.max_(0)), surface.max_(1) + 0.1 + 0.2, 0) *
      //     Eigen::AngleAxisd(1.5 * M_PI, Eigen::Vector3d::UnitZ()));
      // local_poses.push_back(Eigen::Translation3d(surface.max_(0) + 0.1 + robot_workspace_min_radius_,
      //                                            0.5 * (surface.min_(1) + surface.max_(1)), 0) *
      //                       Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      // local_poses.push_back(Eigen::Translation3d(0.5 * (surface.min_(0) + surface.max_(0)),
      //                                            surface.min_(1) - 0.1 - robot_workspace_min_radius_, 0) *
      //                       Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
      // local_poses.push_back(
      //     Eigen::Translation3d(0.5 * (surface.min_(0) + surface.max_(0)), surface.min_(1) - 0.1 - 0.2, 0) *
      //     Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
      // local_poses.push_back(Eigen::Translation3d(surface.min_(0) - 0.1 - robot_workspace_min_radius_,
      //                                            0.5 * (surface.min_(1) + surface.max_(1)), 0) *
      //                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

      local_poses.push_back(Eigen::Translation3d(surface.min_(0) + 0.3 * (surface.max_(0) - surface.min_(0)),
                                                 surface.max_(1) + 0.1 + robot_workspace_min_radius_, 0) *
                            Eigen::AngleAxisd(1.5 * M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(Eigen::Translation3d(surface.min_(0) + 0.7 * (surface.max_(0) - surface.min_(0)),
                                                 surface.max_(1) + 0.1 + robot_workspace_min_radius_, 0) *
                            Eigen::AngleAxisd(1.5 * M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(Eigen::Translation3d(surface.max_(0) + 0.1 + robot_workspace_min_radius_,
                                                 0.5 * (surface.min_(1) + surface.max_(1)), 0) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(Eigen::Translation3d(surface.min_(0) + 0.3 * (surface.max_(0) - surface.min_(0)),
                                                 surface.min_(1) - 0.1 - robot_workspace_min_radius_, 0) *
                            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(Eigen::Translation3d(surface.min_(0) + 0.7 * (surface.max_(0) - surface.min_(0)),
                                                 surface.min_(1) - 0.1 - robot_workspace_min_radius_, 0) *
                            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(
          Eigen::Translation3d(0.5 * (surface.min_(0) + surface.max_(0)), surface.min_(1) - 0.1 - 0.2, 0) *
          Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
      local_poses.push_back(Eigen::Translation3d(surface.min_(0) - 0.1 - robot_workspace_min_radius_,
                                                 0.5 * (surface.min_(1) + surface.max_(1)), 0) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

      for (const auto &local_pose : local_poses) {
        Eigen::Affine3d affine = object.second.pose_ * surface.pose_ * local_pose;
        double x = affine.translation()(0);
        double y = affine.translation()(1);
        affine.translation()(2) = 0;
        double yaw = Eigen::AngleAxisd(affine.rotation()).angle();
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

  allowed_distance_between_connected_locations_ = 0;
  for (const auto &object_distances : object_to_origin_distances) {
    double max = *std::max_element(object_distances.second.begin(), object_distances.second.end());
    if (max > allowed_distance_between_connected_locations_)
      allowed_distance_between_connected_locations_ = max;
  }
  // allowed_distance_between_connected_locations_ += 1e-3;
  allowed_distance_between_connected_locations_ += 2e-1;
  // TODO: provisional min allowd distance thresshold

  allowed_distance_between_connected_locations_ =
      (allowed_distance_between_connected_locations_ > nh_.param("min_base_allowed_distance", 1.5))
          ? allowed_distance_between_connected_locations_
          : nh_.param("min_base_allowed_distance", 1.5);

  std::cout << "Computed max travelling distance " << allowed_distance_between_connected_locations_ << std::endl;
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
    hasher_.CombineHashPose(hash, base_locations_.at(i));
    location_connections_.insert(std::make_pair(hash, connections));
  }
}

bool MoveitTampProblem::LocationReachable(const Eigen::Affine3d &origin, const Eigen::Affine3d &destination) const {

  // return Eigen::Vector3d(destination.translation() - origin.translation()).squaredNorm() <=
  //        allowed_distance_between_connected_locations_ * allowed_distance_between_connected_locations_;
  // TODO: provional check to avoid wrong connections side vs side table bases
  if (exclude_fronted_locations_) {
    double angle_diff =
        acos(cos((destination.rotation().eulerAngles(0, 1, 2)[2] - origin.rotation().eulerAngles(0, 1, 2)[2])));

    return (Eigen::Vector3d(destination.translation() - origin.translation()).squaredNorm() <=
            allowed_distance_between_connected_locations_ * allowed_distance_between_connected_locations_) &&
           (fabs(angle_diff - M_PI) > 1e-3);
  } else {
    return Eigen::Vector3d(destination.translation() - origin.translation()).squaredNorm() <=
           allowed_distance_between_connected_locations_ * allowed_distance_between_connected_locations_;
  }
}

// void MoveitTampProblem::LoadPlacements(std::size_t num_surfaces) {
//   std::size_t num_placements = nh_.param("num_placements", 100 - 2 * 14);

//   std::vector<double> weights(num_surfaces);
//   for (unsigned int k = 0; k < num_placements; ++k) {
//     unsigned int i = 0;
//     for (const auto &object : objects_) {
//       for (const auto &surface : object.second.surfaces_) {
//         weights.at(i++) = surface.area() / (1e-6 + double(surface.placements_.size()));
//       }
//     }

//     std::discrete_distribution<std::size_t> distribution(weights.begin(), weights.end());
//     unsigned int j = distribution(generator_);
//     i = 0;
//     for (auto &object : objects_) {

//       if (i + object.second.surfaces_.size() <= j) {
//         i += object.second.surfaces_.size();
//       } else {
//         object.second.surfaces_.at(j - i).sample();
//         break;
//       }
//     }
//   }
//   std::cout << "Loaded Placements" << std::endl;
// }

void MoveitTampProblem::LoadPlacements(std::size_t num_surfaces) {
  std::size_t num_placements = nh_.param("num_placements", 100);

  for (auto &object : objects_) {
    for (auto &surface : object.second.surfaces_) {
      while (surface.placements_.size() * num_surfaces < num_placements) {
        surface.sample();
      }
    }
  }
  std::cout << "Loaded Placements" << std::endl;
}

void MoveitTampProblem::AdaptativeSampling(const State *const state) {
  // Clear
  for (const auto &name : supporting_object_names) {
    auto supporting_obj = objects_.find(name);
    if (supporting_obj != objects_.end()) {
      for (auto &surface : supporting_obj->second.surfaces_) {
        surface.placements_.clear();
      }
    }
  }

  auto casted_state = dynamic_cast<const MoveitTampState *>(state);
  AddCurrentObjectsToPlacements(casted_state->GetObjectPoses());
  // std::cout<< "There are "<<goal_positions_.size()<<" goal positions"<<std::endl;
  // Add Non-Occupied target poses
  for (const auto &target_pose : goal_positions_) {
    // std::cout<<"target pose "<<target_pose.first<<std::endl;
    bool surface_found = false;
    for (const auto &name : supporting_object_names) {
      if (surface_found)
        break;
      auto supporting_obj = objects_.find(name);
      if (supporting_obj != objects_.end()) {
        for (auto &surface : supporting_obj->second.surfaces_) {
          if (surface_found)
            break;
          if (surface.on(supporting_obj->second.pose_.inverse() * target_pose.second)) {
            surface_found = true;

            bool placement_found = false;
            for (const auto &placement : surface.placements_) {
              if (placement_found)
                break;
              if (OnCircle(target_pose.second, goal_tolerance_radius_,
                           supporting_obj->second.pose_ * placement.translation())) {
                placement_found = true;
              }
            }
            if (!placement_found) {
              // std::cout<<"added to placements"<<std::endl;
              surface.placements_.push_back(supporting_obj->second.pose_.inverse() *
                                            Eigen::Affine3d(Eigen::Translation3d(target_pose.second)));
            }
            // else{
            //   std::cout << "NOT added to placements" << std::endl;
            // }
          }
        }
      }
    }
  }

  for (const auto &name : supporting_object_names) {
    auto supporting_obj = objects_.find(name);
    if (supporting_obj != objects_.end()) {
      for (auto &surface : supporting_obj->second.surfaces_) {
        surface.sample(placements_per_table_adaptative_sampling_);
      }
    }
  }

  display_placements_.poses.clear();
  for (const auto &name : supporting_object_names) {
    auto supporting_obj = objects_.find(name);
    if (supporting_obj != objects_.end()) {
      for (const auto &surface : supporting_obj->second.surfaces_) {
        for (const auto &placement : surface.placements_) {
          geometry_msgs::Pose pose;
          tf::poseEigenToMsg(supporting_obj->second.pose_ * placement, pose);
          display_placements_.poses.push_back(pose);
        }
      }
    }
  }
  const auto object_poses = casted_state->GetObjectPoses();
  assert(object_poses.size() == object_idx_to_marker_idx_.size());
  geometry_msgs::Pose pose_msg;
  for (std::size_t i = 0; i < object_poses.size(); i++) {
    tf::poseEigenToMsg(object_poses.at(i), pose_msg);
    display_objects_.markers.at(object_idx_to_marker_idx_.at(i)).pose = pose_msg;
  }
  // std::cout<<"end of adaptative sampling"<<std::endl;
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
  if (nh_.param("rviz_visualization", false)) {
    pub_placements_.publish(display_placements_);
    pub_locations_.publish(display_locations_);
    pub_objects_.publish(display_objects_);
    pub_robot_state_.publish(display_robot_state_);
  }
  static tf::Transform transform;
  tf::transformEigenToTF(robot_pose_, transform);
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), common_reference_, move_group_interface_.getPlanningFrame()));
}

// Motion planning related methods
bool MoveitTampProblem::ComputeIK(
    const geometry_msgs::Pose &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal,
    const moveit_msgs::AttachedCollisionObject &attached_object = moveit_msgs::AttachedCollisionObject()) {
  num_total_ik_calls_++;
  // std::cout << "In compute IK" << std::endl;
  moveit_msgs::RobotState::_joint_state_type temp_solution;
  std::string x;
  // Reuse srv request and change the goal pose
  srv_.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  srv_.request.ik_request.pose_stamped.pose = pose_goal;
  // srv_.request.ik_request.robot_state.joint_state.position = move_group_interface_.getRandomJointValues();
  if (!attached_object.link_name.empty()) {
    srv_.request.ik_request.robot_state.attached_collision_objects.push_back(attached_object);
  }

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
      // TODO: provisional solution while bug "ik service is returning the whole robot joints instead of only the
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
      display_robot_state_.state = srv_.request.ik_request.robot_state;
      display_robot_state_.state.joint_state = joint_goal;
      // std::cout << "type any key to continue" << std::endl;
      // std::cin >> x;
      num_successful_ik_calls_++;
      return true;
    } else {
      // ROS_ERROR("Solution not found");
      display_robot_state_.state = srv_.request.ik_request.robot_state;
      display_robot_state_.state.joint_state = robot_home_joint_config_;
      // std::cout << "type any key to continue" << std::endl;
      // std::cin >> x;
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
    // std::cout << "Object " << object_name << "starts at " << objects_.at(object_name).pose_.translation().transpose()
    // << std::endl;
    object_poses.push_back(objects_.at(object_name).pose_);
  }

  std::size_t state_hash;
  std::size_t state_local_hash;
  std::vector<std::size_t> on_workspace_objects;
  std::vector<std::size_t> features;
  ComputeHashes(robot_origin_, object_poses, "", state_hash, state_local_hash, on_workspace_objects, features);

  return new MoveitTampState(robot_origin_, object_poses, state_hash, state_local_hash, on_workspace_objects, features);
}

bool MoveitTampProblem::ComputeIK(
    const Eigen::Affine3d &pose_goal, moveit_msgs::RobotState::_joint_state_type &joint_goal,
    const moveit_msgs::AttachedCollisionObject &attached_object = moveit_msgs::AttachedCollisionObject()) {
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose_goal, pose_msg);
  return ComputeIK(pose_msg, joint_goal, attached_object);
}
void MoveitTampProblem::InitIKRequest(moveit_msgs::GetPositionIK &srv) {
  srv.request.ik_request.group_name = move_group_interface_.getName();
  // std::cout << "Cooking ik request for planning group" << srv.request.ik_request.group_name << std::endl;
  srv.request.ik_request.robot_state.joint_state.name = move_group_interface_.getJointNames();
  srv.request.ik_request.robot_state.joint_state.position = robot_home_joint_config_.position;
  srv.request.ik_request.pose_stamped.header.frame_id = "base_footprint";
  double ik_timeout = nh_.param("ik_timeout", 0.3);
  srv.request.ik_request.timeout = ros::Duration(ik_timeout);
  srv.request.ik_request.ik_link_name = "arm_tool_link";
  srv.request.ik_request.avoid_collisions = nh_.param("avoid_collisions_in_IK", true);

  // std::cout<<"ik request initialized"<<std::endl;
}
void MoveitTampProblem::InitIKRequest() { InitIKRequest(srv_); }

bool MoveitTampProblem::PlanToJointTarget(const moveit_msgs::RobotState::_joint_state_type &joint_goal,
                                          moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                          const moveit_msgs::RobotState &robot_start_state = moveit_msgs::RobotState())

{
  num_total_motion_plans_++;

  if (robot_start_state.joint_state.position.empty()) {
    move_group_interface_.setStartStateToCurrentState();

    // moveit::core::robotStateToRobotStateMsg(*move_group_interface_.getCurrentState(),display_robot_state_.state);
  } else {
    // move_group_interface_.setStartState(robot_start_state);
    auto current = move_group_interface_.getCurrentState();
    current->setJointGroupPositions("arm_torso", robot_start_state.joint_state.position);

    move_group_interface_.setStartState(*current);

    // prova:
    //  move_group_interface_.setStartStateToCurrentState();
    //  move_group_interface_.setStartState(robot_start_state);

    // moveit::core::robotStateToRobotStateMsg(*current, display_robot_state_.state);
  }

  // std::string x;
  // std::cout << "Start" << std::endl;
  // std::cin >> x;
  // auto current = move_group_interface_.getCurrentState();
  // current->setJointGroupPositions("arm_torso", joint_goal.position);
  // moveit::core::robotStateToRobotStateMsg(*current, display_robot_state_.state);
  // std::cout << "Goal" << std::endl;
  // std::cin >> x;

  auto valid_goal = move_group_interface_.setJointValueTarget(joint_goal);
  assert(valid_goal);

  auto result = move_group_interface_.plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("Planning failed with error code: " << result);
    return false;
  } else {
    ROS_DEBUG_STREAM("Plan found in " << plan.planning_time_ << " seconds");
    num_successful_motion_plans_++;
    return true;
  }
}

Eigen::Affine3d bezier(double u, std::vector<Eigen::Affine3d>::const_iterator begin,
                       std::vector<Eigen::Affine3d>::const_iterator end) {
  if (begin + 1 != end) {
    Eigen::Affine3d pre = bezier(u, begin, end - 1);
    Eigen::Affine3d post = bezier(u, begin + 1, end);
    return Eigen::Affine3d(Eigen::Translation3d(pre.translation() * (1 - u) + post.translation() * u) *
                           (Eigen::Quaterniond(pre.rotation()).slerp(u, Eigen::Quaterniond(post.rotation()))));
  } else {
    return *begin;
  }
}
bool MoveitTampProblem::PlanAndExecuteMoveGripperToNamedTarget(
    moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface, const std::string &named_target) {
  // gripper_move_group_interface.setStartStateToCurrentState();

  moveit_msgs::RobotTrajectory trajectory_close;
  trajectory_close.joint_trajectory.header.stamp = ros::Time::now();
  trajectory_close.joint_trajectory.joint_names.push_back("gripper_left_finger_joint");
  trajectory_close.joint_trajectory.joint_names.push_back("gripper_right_finger_joint");
  moveit_msgs::RobotTrajectory trajectory_open;
  trajectory_open = trajectory_close;
  trajectory_msgs::JointTrajectoryPoint init_point;
  init_point.positions.push_back(0.045);
  init_point.positions.push_back(0.045);
  init_point.time_from_start.fromSec(0);
  // init_point.velocities.push_back(0);
  // init_point.velocities.push_back(0);
  // init_point.accelerations.push_back(0);
  // init_point.accelerations.push_back(0);
  trajectory_close.joint_trajectory.points.push_back(init_point);
  init_point.positions.clear();
  init_point.positions.push_back(0.025);
  init_point.positions.push_back(0.025);
  trajectory_open.joint_trajectory.points.push_back(init_point);

  // trajectory_msgs::JointTrajectoryPoint mid_point;
  // mid_point.positions.push_back(0.035);
  // mid_point.positions.push_back(0.035);
  // mid_point.time_from_start.fromSec(0.5);
  // mid_point.velocities.push_back(0.04);
  // mid_point.velocities.push_back(0.04);
  // mid_point.accelerations.push_back(0.02);
  // mid_point.accelerations.push_back(0.02);

  // trajectory_close.joint_trajectory.points.push_back(mid_point);
  // trajectory_open.joint_trajectory.points.push_back(mid_point);

  trajectory_msgs::JointTrajectoryPoint end_point;
  end_point.positions.push_back(0.025);
  end_point.positions.push_back(0.025);
  end_point.time_from_start.fromSec(1);
  // end_point.velocities.push_back(0);
  // end_point.velocities.push_back(0);
  // end_point.accelerations.push_back(0);
  // end_point.accelerations.push_back(0);
  trajectory_close.joint_trajectory.points.push_back(end_point);
  end_point.positions.clear();
  end_point.positions.push_back(0.045);
  end_point.positions.push_back(0.045);
  trajectory_open.joint_trajectory.points.push_back(end_point);
  moveit::core::MoveItErrorCode result;
  if (named_target == "grasp") {
    result = gripper_move_group_interface.execute(trajectory_close);
  } else {
    result = gripper_move_group_interface.execute(trajectory_open);
  }

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    return true;
  } else {
    ROS_ERROR_STREAM("Failed to move gripper with error code: " << result);
    return false;
  }

  // gripper_move_group_interface.setStartStateToCurrentState();
  // auto named_target_res = gripper_move_group_interface.setNamedTarget(named_target);
  // if (named_target_res) {

  //   gripper_move_group_interface.execute() auto result = gripper_move_group_interface.move();
  //   if (result == moveit::core::MoveItErrorCode::SUCCESS) {
  //     return true;
  //   } else {
  //     ROS_ERROR_STREAM("Failed to move gripper with error code: " << result);
  //     return false;
  //   }
  // } else {
  //   ROS_ERROR_STREAM("Invalid named target for gripper group: " << named_target);
  //   return false;
  // }
}
bool MoveitTampProblem::PlanAndExecuteCloseGripper(
    moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface) {
  return PlanAndExecuteMoveGripperToNamedTarget(gripper_move_group_interface, "grasp");
}
bool MoveitTampProblem::PlanAndExecuteOpenGripper(
    moveit::planning_interface::MoveGroupInterface &gripper_move_group_interface) {
  return PlanAndExecuteMoveGripperToNamedTarget(gripper_move_group_interface, "open");
}

bool MoveitTampProblem::ExecutePlan(const Plan &plan) {
  // TODO make robot be at start state

  moveit::planning_interface::MoveGroupInterface gripper_move_group_interface("gripper");
  gripper_move_group_interface.setMaxVelocityScalingFactor(1);
  gripper_move_group_interface.setMaxAccelerationScalingFactor(1);
  // if (!PlanAndExecuteOpenGripper(gripper_move_group_interface)) {
  //   return false;
  // }
  robot_pose_ = robot_origin_;
  // std::size_t j = 0;
  // for (std::size_t i = 0; i < objects_.size(); ++i) {
  //   while (display_objects_.markers.at(j).ns.find("moveable_objects") == std::string::npos)
  //     j++;
  //   display_objects_.markers.at(j).header.frame_id = common_reference_;
  //   tf::poseEigenToMsg(objects_.at(object_names_.at(i)).pose_, display_objects_.markers.at(j).pose);
  //   j++;
  // }
  display_objects_ = display_objects_start_;
  std::cout << "Be ready for plan execution" << std::endl;
  std::string x;
  std::cin >> x;

  bool first_move_base = true;
  std::vector<Eigen::Affine3d> points;
  for (const auto &action : plan.actions) {
    if (auto move_base_action = dynamic_cast<const MoveBaseAction *const>(action)) {
      if (points.empty()) {
        points.push_back(robot_pose_);
        if (first_move_base) {
          first_move_base = false;
        } else {
          points.push_back(robot_pose_ * Eigen::Translation3d(-1, 0, 0));
        }
      }
      auto via_points = move_base_action->GetViaPoints();
      points.insert(points.end(), via_points.begin(), via_points.end());
      points.push_back(move_base_action->GetTargetLocation());
    } else {
      if (!points.empty()) {
        points.push_back(points.back());
        points.at(points.size() - 2) = points.at(points.size() - 2) * Eigen::Translation3d(-1, 0, 0);

        double dist = 0;
        for (std::size_t i = 1; i < points.size(); ++i) {
          dist += (points.at(i).translation() - points.at(i - 1).translation()).norm();
        }
        double robot_vel = 1.;
        double delta_t = 0.01;
        std::size_t num_steps = ceil(dist / (robot_vel * delta_t));
        for (std::size_t k = 0; k <= num_steps; ++k) {
          double u = double(k) / double(num_steps);
          robot_pose_ = bezier(u, points.cbegin(), points.cend());
          ros::Duration(delta_t).sleep();
        }
        points.clear();
      }
      if (auto pick_action = dynamic_cast<const PickAction *const>(action)) {
        if (!ExecutePlan(pick_action->GetToObjectPlan()))
          return false;
        auto &marker = display_objects_.markers.at(object_indices.at(pick_action->GetTargetObjectId()));
        marker.header.frame_id = move_group_interface_.getEndEffectorLink();
        tf::poseEigenToMsg(pick_action->GetGrasp()->inverse(), marker.pose);
        if (!PlanAndExecuteCloseGripper(gripper_move_group_interface)) {
          return false;
        }
        if (!ExecutePlan(pick_action->GetToHomePlan()))
          return false;
      } else if (auto place_action = dynamic_cast<const PlaceAction *const>(action)) {
        if (!ExecutePlan(place_action->GetToObjectPlan()))
          return false;
        auto &marker = display_objects_.markers.at(object_indices.at(place_action->GetTargetObjectId()));
        marker.header.frame_id = common_reference_;
        tf::poseEigenToMsg(place_action->GetTargetObjectPose(), marker.pose);
        if (!PlanAndExecuteOpenGripper(gripper_move_group_interface)) {
          return false;
        }
        if (!ExecutePlan(place_action->GetToHomePlan()))
          return false;
      } else {
        ROS_ERROR("UNRECOGNIZED ACTION CLASS");
        return false;
      }
    }
  }
  if (!points.empty()) {
    points.push_back(points.back());
    points.at(points.size() - 2) = points.at(points.size() - 2) * Eigen::Translation3d(0, 0, -1);

    double dist = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      dist += (points.at(i).translation() - points.at(i - 1).translation()).norm();
    }
    double robot_vel = 1.;
    double delta_t = 0.1;
    std::size_t num_steps = ceil(dist / (robot_vel * delta_t));
    for (std::size_t k = 0; k <= num_steps; ++k) {
      double u = double(k) / double(num_steps);
      robot_pose_ = bezier(u, points.cbegin(), points.cend());
      ros::Duration(delta_t).sleep();
    }
  }

  return true;
}

bool MoveitTampProblem::ExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
  std::cout << "Plan will be executed now" << std::endl;
  std::cout << "Plan has " << plan.trajectory_.joint_trajectory.points.size() << " points" << std::endl;
  std::cout << "Plan lasts " << plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() << " seconds"
            << std::endl;
  auto result = move_group_interface_.execute(plan);
  std::cout << "Plan returned " << result << std::endl;
  return (result == result.SUCCESS);
}

// bool MoveitTampProblem::IsGoal(State const *const state) const {
//   // Goal = for all X, stick_blueX on table3 and stick_greenX on table4
//   // TODO: Goal hardcode only for this PoC
//   auto casted_state = dynamic_cast<const MoveitTampState *>(state);
//   const auto &table3 = objects_.at("table3");
//   const auto table_3_pose_inv = table3.pose_.inverse();
//   const auto &table4 = objects_.at("table4");
//   const auto table_4_pose_inv = table4.pose_.inverse();

//   for (std::size_t i = 0; i < objects_.size(); ++i) {
//     if (object_names_.at(i).find("stick_blue") != std::string::npos &&
//         !table3.surfaces_.front().on(table_3_pose_inv * casted_state->GetObjectPoses().at(i).translation())) {
//       return false;
//     }
//     if (object_names_.at(i).find("stick_green") != std::string::npos &&
//         !table4.surfaces_.front().on(table_4_pose_inv * casted_state->GetObjectPoses().at(i).translation())) {
//       return false;
//     }
//   }

//   return true;
// }

void MoveitTampProblem::ComputeStateSketchFeatures(State *const state) {

  // compute state features
  auto casted_state = dynamic_cast<MoveitTampState *>(state);
  SetMisplacedObjects(casted_state);
  // std::cout<<"computed misplaced of start state"<<std::endl;
  SetBlockingObjects(casted_state, true);
  // std::cout << "computed blocking of start state" << std::endl;
}
void MoveitTampProblem::SetMisplacedObjects(MoveitTampState *const state) const {

  std::unordered_set<std::size_t> state_misplaced_objects;
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    if (state->GetAttatchedObject() != object_names_.at(i) &&
        Misplaced(object_names_.at(i), state->GetObjectPoses().at(i)))
      state_misplaced_objects.insert(i);
  }

  state->SetMisplacedObjects(state_misplaced_objects);
}

bool MoveitTampProblem::IsGoal(State const *const state) const {

  if (active_sketch_rule_ == MoveitTampProblem::END) {
    std::cout << "FINAL GOAL FOUND" << std::endl;
    return true; // State is a final goal
  }
  auto state_copy = state->Clone(); // TODO: Alguna manera més elegant?
  auto casted_state = dynamic_cast<MoveitTampState *>(state_copy);
  bool H = casted_state->HasObjectAttached();
  std::size_t m, n, s;
  // bool F;

  switch (active_sketch_rule_) {
  case MoveitTampProblem::PICK_MISPLACED_OBJECT:
    if (!H) {
      delete state_copy;
      return false;
    }
    SetMisplacedObjects(casted_state);
    m = casted_state->GetNumOfMisplacedObjects();

    // F = !AllGoalRegionBlockMisplaced(casted_state->GetAttatchedObject(), casted_state);
    // if (m < start_state_sketch_features_.m) {
    //   std::cout << "Stopped in br: Grasped object is: " << casted_state->GetAttatchedObject()
    //   << " F: " << F
    //             << " m_init, m_current: " << start_state_sketch_features_.m << "," << m << std::endl;
    //   char c;
    //   std::cin >> c;
    // }
    if (m < start_state_sketch_features_.m) {

      std::cout << "PICK_MISPLACED_OBJECT SUBGOAL FOUND" << std::endl;
      std::cout << "H is now: " << H << "F is now: " << true << " and m decremented from "
                << start_state_sketch_features_.m << " to " << m << std::endl;
      delete state_copy;
      return true;
    } else {
      delete state_copy;
      return false;
    }
    break;

  case MoveitTampProblem::PICK_OBSTRUCTING_OBJECT:
    if (!H) {
      delete state_copy;
      return false;
    }
    SetMisplacedObjects(casted_state);
    SetBlockingObjects(casted_state, true);
    s = casted_state->GetSumMinObjectsObstructingMisplacedObjects();
    std::cout << "Trying to pick object " << casted_state->GetAttatchedObject() << " s changes from "
              << start_state_sketch_features_.s << " to " << s << std::endl;

    if (s < start_state_sketch_features_.s) {
      std::cout << "PICK_OBSTRUCTING_OBJECT SUBGOAL FOUND" << std::endl;
      std::cout << "H is now: " << H << " and s decremented from " << start_state_sketch_features_.s << " to " << s
                << std::endl;
      delete state_copy;
      return true;
    } else {
      delete state_copy;
      return false;
    }
    break;

  case MoveitTampProblem::PLACE_OBJECT_FREE:
    if (H) {
      delete state_copy;
      return false;
    }
    SetMisplacedObjects(casted_state);
    m = casted_state->GetNumOfMisplacedObjects();
    if (m != start_state_sketch_features_.m) {
      delete state_copy;
      // std::cout << "Discarted place free object because m changes from " << start_state_sketch_features_.m << " to "
      //           << m << std::endl;
      return false;
    }
    SetBlockingObjects(casted_state, true);
    n = casted_state->GetMinObstructingObjects();
    s = casted_state->GetSumMinObjectsObstructingMisplacedObjects();
    if (n == start_state_sketch_features_.n && s == start_state_sketch_features_.s) {
      std::cout << "PLACE_OBJECT_FREE SUBGOAL FOUND" << std::endl;
      std::cout << "H is now: " << H
                << " and features m,n,s keep its values (start values: " << start_state_sketch_features_.m << ", "
                << start_state_sketch_features_.n << ", " << start_state_sketch_features_.s << ") (end values: " << m
                << ", " << n << ", " << s << ")" << std::endl;
      delete state_copy;
      return true;
    } else {
      // if (n != start_state_sketch_features_.n)
      //   std::cout << "Discarted place free object because n changes from " << start_state_sketch_features_.n << " to
      //   "
      //             << n << std::endl;

      // if (s != start_state_sketch_features_.s)
      //   std::cout << "Discarted place free object because s changes from " << start_state_sketch_features_.s << " to
      //   "
      //             << s << std::endl;
      delete state_copy;
      return false;
    }
    break;
  case MoveitTampProblem::PLACE_OBJECT_BLOCKED:
    if (H) {
      delete state_copy;
      return false;
    }
    SetMisplacedObjects(casted_state);
    // m = casted_state->GetNumOfMisplacedObjects();
    // if (m != start_state_sketch_features_.m) {
    //   delete state_copy;
    //   return false;
    // }
    SetBlockingObjects(casted_state, true);
    n = casted_state->GetMinObstructingObjects();
    s = casted_state->GetSumMinObjectsObstructingMisplacedObjects();
    if (s == start_state_sketch_features_.s) {
      // if (n == start_state_sketch_features_.n && s == start_state_sketch_features_.s) {
      std::cout << "PLACE_OBJECT_BLOCKED SUBGOAL FOUND" << std::endl;
      std::cout << "H is now: " << H
                << " and features n,s keep its values (start values: " << start_state_sketch_features_.n << ", "
                << start_state_sketch_features_.s << ") (end values: " << n << ", " << s << ")" << std::endl;
      delete state_copy;
      return true;
    } else {
      // if (n != start_state_sketch_features_.n)
      //   std::cout << "Discarted place blocked object because n changes from " << start_state_sketch_features_.n
      //             << " to " << n << std::endl;

      if (s != start_state_sketch_features_.s)
        std::cout << "Discarted place blocked object because s changes from " << start_state_sketch_features_.s
                  << " to " << s << std::endl;
      delete state_copy;
      return false;
    }
    break;

  default:
    delete state_copy;
    return false;
    break;
  }

  // Goal = for all X, stick_blueX on table3 or stick_greenX on table4

  // const auto &table3 = objects_.at("table3");
  // const auto table_3_pose_inv = table3.pose_.inverse();
  // const auto &table4 = objects_.at("table4");
  // const auto table_4_pose_inv = table4.pose_.inverse();

  // for (std::size_t i = 0; i < objects_.size(); ++i) {
  //   if (object_names_.at(i).find("stick_blue") != std::string::npos &&
  //       table3.surfaces_.front().on(table_3_pose_inv * casted_state->GetObjectPoses().at(i).translation())) {
  //     return true;
  //   }
  //   if (object_names_.at(i).find("stick_green") != std::string::npos &&
  //       table4.surfaces_.front().on(table_4_pose_inv * casted_state->GetObjectPoses().at(i).translation())) {
  //     return true;
  //   }
  // }

  // return false;
}

void MoveitTampProblem::MoveCollisionObject(const std::string &obj_id, const geometry_msgs::Pose &new_pose,
                                            const std::string &new_reference_frame = "") {
  planning_scene_interface_.applyCollisionObject(GenerateMoveCollisionObjectMsg(obj_id, new_pose, new_reference_frame));
}

void MoveitTampProblem::MoveCollisionObject(const std::string &obj_id, const Eigen::Affine3d &new_pose,
                                            const std::string &new_reference_frame = "") {
  geometry_msgs::Pose new_pose_msg;
  tf::poseEigenToMsg(new_pose, new_pose_msg);
  MoveCollisionObject(obj_id, new_pose_msg, new_reference_frame);
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
      collision_objects.push_back(
          GenerateMoveCollisionObjectMsg(obj_ids.at(i), new_poses.at(i), move_group_interface_.getPlanningFrame()));
    }
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);
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
      collision_objects.push_back(
          GenerateMoveCollisionObjectMsg(obj_ids.at(i), pose, move_group_interface_.getPlanningFrame()));
    }
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);
}

moveit_msgs::CollisionObject MoveitTampProblem::GenerateMoveCollisionObjectMsg(const std::string &obj_id,
                                                                               const geometry_msgs::Pose &new_pose,
                                                                               const std::string &new_reference_frame) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = obj_id;
  collision_object.operation = moveit_msgs::CollisionObject::MOVE;
  collision_object.header.frame_id = new_reference_frame;
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
  return (OnCircle(robot_workspace_center_translation_, robot_workspace_max_radius_,
                   (robot_base_pose.inverse() * target_pose).translation()) &&
          !OnCircle(robot_workspace_center_translation_, robot_workspace_min_radius_,
                    (robot_base_pose.inverse() * target_pose).translation()));
  // TODO: Optimize OnWorkspace combining in a single function OnAnnulus
}
bool MoveitTampProblem::OnCircle(const Eigen::Vector3d &origin, const double radius,
                                 const Eigen::Vector3d &target) const {

  // NOTE: World coordinates assumed to have z inverse to gravity direction. Assumed that robot can lift its first
  // joint in order to have each maximum reachability distance at the height of the object
  const auto dx = (target(0) > origin(0)) ? (target(0) - origin(0)) : (origin(0) - target(0));
  if (dx > radius)
    return false;
  const auto dy = (target(1) > origin(1)) ? (target(1) - origin(1)) : (origin(1) - target(1));
  if (dy > radius)
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
  assert(casted_state);
  if (reuse_expansions_) {
    auto actions_iter = discovered_valid_actions_.find(casted_state->GetHash());
    if (actions_iter != discovered_valid_actions_.end()) {
      reused_valid_actions_ += actions_iter->second.size();
      return actions_iter->second;
    }
  }

  std::vector<Action *> valid_actions;
  const Eigen::Affine3d robot_base_tf_inverse = casted_state->GetRobotBasePose().inverse();

  auto object_poses_in_robot_coords = casted_state->GetObjectPoses();
  for (auto &pose : object_poses_in_robot_coords)
    pose = robot_base_tf_inverse * pose;
  MoveCollisionObjects(object_names_, object_poses_in_robot_coords);

  // PLACE OR PICK ACTIONS DEPENDING ON IF THE ROBOT HAVE AN ATTACHED OBJECT
  const auto obj_poses = casted_state->GetObjectPoses();
  if (casted_state->HasObjectAttached()) {

    const auto attached_object = objects_.find(casted_state->GetAttatchedObject());
    assert(attached_object == objects_.end());
    // move_group_interface_.attachObject(attached_object->first);

    // moveit_msgs::AttachedCollisionObject attached_collision_object;
    // attached_collision_object.link_name = robot_grasping_link_;
    // geometry_msgs::Pose grasping_link_to_obj;
    // tf::poseEigenToMsg(casted_state->GetGrasp()->inverse(), grasping_link_to_obj);
    // attached_collision_object.object =
    //     GenerateMoveCollisionObjectMsg(attached_object->first, grasping_link_to_obj, robot_grasping_link_);

    // PLACE ACTIONS
    for (const auto &object : objects_) {
      for (const auto &surface : object.second.surfaces_) {
        for (const auto &placement : surface.placements_) {
          // std::cout << "Checking Workspace in placement" << std::endl;

          if (!OnWorkspace(casted_state->GetRobotBasePose(), object.second.pose_ * placement))
            continue;

          // TODO assuming placement belongs to a non-movable object. This assumption is repeated in whole code
          const auto placement_pos = object.second.pose_ * placement.translation();
          bool found = false;
          auto it = obj_poses.begin();
          while (!found && it != obj_poses.end()) {
            // TODO Solve semiamplitude issue
            // found = OnCircle(placement_pos, gripper_semiamplitude_, it->translation());
            found = OnCircle(placement_pos,
                             (casted_state->GetAttatchedObject().find("red") != std::string::npos) ? 0.12 : 0.045,
                             it->translation());
            if (!found)
              ++it;
          }
          if (found)
            continue;

          for (const auto stable_object_pose : attached_object->second.stable_object_poses_) {
            static moveit_msgs::RobotState::_joint_state_type joint_goal;
            if (!ComputeIK(robot_base_tf_inverse * object.second.pose_ * placement * stable_object_pose *
                               (*casted_state->GetGrasp()),
                           joint_goal))
              continue;
            // std::cout << "creating PlaceAction" << std::endl;
            auto action = new PlaceAction(attached_object->first, joint_goal,
                                          object.second.pose_ * placement * stable_object_pose);
            if (!lazy && !IsActionValid(state, action)) {
              delete action;
              continue;
            }
            num_place_++;
            valid_actions.push_back(action);
          }
        }
      }
    }
  } else {
    for (const auto i : casted_state->GetOnWorkspaceObjects()) {

      const auto object = objects_.find(object_names_.at(i));
      assert(object == objects_.end());

      if (!object->second.moveable_)
        continue;

      for (const auto &grasp : object->second.grasps_) {
        static moveit_msgs::RobotState::_joint_state_type joint_goal;
        if (!ComputeIK(robot_base_tf_inverse * obj_poses.at(i) * grasp, joint_goal))
          continue;

        auto action = new PickAction(object->first, joint_goal, &grasp, obj_poses.at(i));
        if (!lazy && !IsActionValid(state, action)) {
          delete action;
          continue;
        }
        num_pick_++;
        valid_actions.push_back(action);
      }
    }
  }

  // MOVE BASE ACTIONS
  std::size_t hash = 0;
  hasher_.CombineHashPose(hash, casted_state->GetRobotBasePose());
  auto res = location_connections_.find(hash);
  assert(res == location_connections_.end());
  for (const auto &target : res->second) {
    auto action = new MoveBaseAction(base_locations_.at(target));
    num_move_base_++;
    valid_actions.push_back(action);
  }

  discovered_valid_actions_.insert(std::make_pair(casted_state->GetHash(), valid_actions));

  return valid_actions;
}

bool MoveitTampProblem::IsActionValid(State const *const state, Action *const action, bool lazy) {
  if (lazy) {
    return true;
  } else if (auto move_base_action = dynamic_cast<MoveBaseAction *>(action)) {
    return true;
  } else if (auto pick_action = dynamic_cast<PickAction *>(action)) {
    /*
                                            ****
                                            PICK
                                            ****
*/
    // In addition, checks if non-lazy:
    auto casted_state = dynamic_cast<const MoveitTampState *>(state);
    const Eigen::Affine3d robot_base_tf_inverse = casted_state->GetRobotBasePose().inverse();

    auto object_poses_in_robot_coords = casted_state->GetObjectPoses();
    for (auto &pose : object_poses_in_robot_coords)
      pose = robot_base_tf_inverse * pose;
    MoveCollisionObjects(object_names_, object_poses_in_robot_coords);

    // Valid trajecory from home to (joint) pick pose
    moveit::planning_interface::MoveGroupInterface::Plan to_object_plan;
    if (!PlanToJointTarget(pick_action->GetJointGoal(), to_object_plan)) {
      return false;
    }
    bool attached = move_group_interface_.attachObject(pick_action->GetTargetObjectId());

    assert(attached);
    // std::string text;
    // std::cout << "Pre planning trajectory" << std::endl;
    // std::cin >> text;
    moveit_msgs::RobotState start_state;
    moveit_msgs::AttachedCollisionObject attached_object;
    geometry_msgs::Pose obj_pose;
    tf::poseEigenToMsg(robot_base_tf_inverse * pick_action->GetTargetObjectPose(), obj_pose);
    attached_object.object = GenerateMoveCollisionObjectMsg(pick_action->GetTargetObjectId(), obj_pose,
                                                            move_group_interface_.getPlanningFrame());

    attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
    attached_object.link_name = robot_grasping_link_;
    // bool attached = planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    // assert(attached);
    start_state.attached_collision_objects.push_back(attached_object);
    start_state.joint_state = pick_action->GetJointGoal();
    start_state.is_diff = true;
    moveit::planning_interface::MoveGroupInterface::Plan to_home_plan;
    if (!PlanToJointTarget(robot_home_joint_config_, to_home_plan, start_state)) {
      move_group_interface_.detachObject(pick_action->GetTargetObjectId());
      return false;
    }
    move_group_interface_.detachObject(pick_action->GetTargetObjectId());
    pick_action->SetToObjectPlan(to_object_plan);
    pick_action->SetToHomePlan(to_home_plan);
    return true;
  } else if (auto place_action = dynamic_cast<PlaceAction *>(action)) {
    /*
                                            ****
                                            PLACE
                                            ****
*/

    auto casted_state = dynamic_cast<const MoveitTampState *>(state);
    const Eigen::Affine3d robot_base_tf_inverse = casted_state->GetRobotBasePose().inverse();

    // Object placing trajectory
    auto object_poses_in_robot_coords = casted_state->GetObjectPoses();
    for (auto &pose : object_poses_in_robot_coords)
      pose = robot_base_tf_inverse * pose;
    MoveCollisionObjects(object_names_, object_poses_in_robot_coords);

    bool attached = move_group_interface_.attachObject(place_action->GetTargetObjectId());
    assert(attached);
    moveit::planning_interface::MoveGroupInterface::Plan to_object_plan;
    if (!PlanToJointTarget(place_action->GetJointGoal(), to_object_plan)) {
      move_group_interface_.detachObject(place_action->GetTargetObjectId());
      return false;
    }
    move_group_interface_.detachObject(place_action->GetTargetObjectId());

    MoveCollisionObject(place_action->GetTargetObjectId(), robot_base_tf_inverse * place_action->GetTargetObjectPose(),
                        robot_root_tf_);

    // Retirement trajectory
    moveit_msgs::RobotState start_state;
    start_state.joint_state = place_action->GetJointGoal();

    moveit::planning_interface::MoveGroupInterface::Plan to_home_plan;
    if (!PlanToJointTarget(robot_home_joint_config_, to_home_plan, start_state)) {
      return false;
    }

    place_action->SetToObjectPlan(to_object_plan);
    place_action->SetToHomePlan(to_home_plan);
    return true;

  } else {
    ROS_ERROR("Unknown action type while checking action valid");
    return false;
  }
}

State *const MoveitTampProblem::GetSuccessor(State const *const state, Action const *const action) {

  auto casted_state = dynamic_cast<const MoveitTampState *>(state);
  // TODO: Maybe a more elegant way to organize kinds of actions, maybe each action should have an Apply method but
  // then is state dependent...
  std::size_t state_hash;
  std::size_t state_local_hash;
  std::vector<std::size_t> on_workspace_objects;
  std::vector<std::size_t> features;
  if (auto move_base_action = dynamic_cast<const MoveBaseAction *>(action)) {
    if (casted_state->HasObjectAttached()) {
      std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
      new_object_poses.at(object_indices.at(casted_state->GetAttatchedObject())) =
          move_base_action->GetTargetLocation() * casted_state->GetRobotBasePose().inverse() *
          new_object_poses.at(object_indices.at(casted_state->GetAttatchedObject()));

      ComputeHashes(move_base_action->GetTargetLocation(), new_object_poses, casted_state->GetAttatchedObject(),
                    state_hash, state_local_hash, on_workspace_objects, features);

      return new MoveitTampState(move_base_action->GetTargetLocation(), new_object_poses, state_hash, state_local_hash,
                                 on_workspace_objects, features, casted_state->GetAttatchedObject(),
                                 casted_state->GetGrasp());

    } else {
      ComputeHashes(move_base_action->GetTargetLocation(), casted_state->GetObjectPoses(), "", state_hash,
                    state_local_hash, on_workspace_objects, features);

      return new MoveitTampState(move_base_action->GetTargetLocation(), casted_state->GetObjectPoses(), state_hash,
                                 state_local_hash, on_workspace_objects, features);
    }

  } else if (auto pick_action = dynamic_cast<const PickAction *>(action)) {
    std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
    new_object_poses.at(object_indices.at(pick_action->GetTargetObjectId())) =
        casted_state->GetRobotBasePose() * gripper_home_wrt_robot_base * pick_action->GetGrasp()->inverse();
    ComputeHashes(casted_state->GetRobotBasePose(), new_object_poses, pick_action->GetTargetObjectId(), state_hash,
                  state_local_hash, on_workspace_objects, features);

    return new MoveitTampState(casted_state->GetRobotBasePose(), new_object_poses, state_hash, state_local_hash,
                               on_workspace_objects, features, pick_action->GetTargetObjectId(),
                               pick_action->GetGrasp());

  } else if (auto place_action = dynamic_cast<const PlaceAction *>(action)) {
    std::vector<Eigen::Affine3d> new_object_poses = casted_state->GetObjectPoses();
    new_object_poses.at(object_indices.at(place_action->GetTargetObjectId())) = place_action->GetTargetObjectPose();
    ComputeHashes(casted_state->GetRobotBasePose(), new_object_poses, "", state_hash, state_local_hash,
                  on_workspace_objects, features);

    return new MoveitTampState(casted_state->GetRobotBasePose(), new_object_poses, state_hash, state_local_hash,
                               on_workspace_objects, features);
  } else {
    ROS_ERROR("UNRECOGNIZED ACTION CLASS");
    return nullptr;
  }
}

void MoveitTampProblem::ComputeHashes(const Eigen::Affine3d &base_pose,
                                      const std::vector<Eigen::Affine3d> &object_poses,
                                      const std::string &attached_object, std::size_t &state_hash,
                                      std::size_t &state_local_hash, std::vector<std::size_t> &on_workspace_objects,
                                      std::vector<std::size_t> &features_hashes) const {
  // Base related hashs
  state_hash = 0;
  state_local_hash = 0;
  hasher_.CombineHashPose(state_hash, base_pose);
  std::size_t hash = 0;
  hasher_.hash_combine<std::string>(hash, std::string("robot"));
  hasher_.CombineHashPose(hash, base_pose);
  features_hashes.push_back(hash);

  // Object related hash
  std::size_t attached_object_index = object_poses.size();
  if (!attached_object.empty()) {
    auto it = object_indices.find(attached_object);
    if (it != object_indices.end()) {
      attached_object_index = it->second;
    }
  }
  for (std::size_t i = 0; i < object_poses.size(); i++) {
    const auto &object = objects_.at(object_names_.at(i));
    if (!object.moveable_)
      continue;

    hasher_.CombineHashPose(state_hash, object_poses.at(i));
    if (OnWorkspace(base_pose, object_poses.at(i))) {
      on_workspace_objects.push_back(i);
      hasher_.hash_combine<std::string>(state_local_hash, object_names_.at(i));
      hasher_.CombineHashPose(state_local_hash, object_poses.at(i));
    }

    hash = 0;
    hasher_.hash_combine<std::string>(hash, object_names_.at(i));
    if (i != attached_object_index) {
      hasher_.CombineHashPose(hash, Eigen::Affine3d(Eigen::Translation3d(object_poses.at(i).translation()) *
                                                    Eigen::Quaterniond::Identity()));
    } else {
      hasher_.CombineHashPose(hash,
                              base_pose); // To represent object grasped it is set as the base pose (it does not add
                                          // unique info to multiply for the tf between the base and the gripper and the
                                          // selected grasped is expressly excluded in order to have more abstraction)
    }
    features_hashes.push_back(hash);
  };
}

bool MoveitTampProblem::SetActiveSketchRule(const State *const state) {
  /*
  SKETCH

  misplaced block = goal block that is a in a wrong table; block is not regarded as "misplaced" if being held

  Features:
         H = true if holding object
         m = number of misplaced blocks
         n = min number of blocks obstructing the misplaced blocks, 0 if m=0
         s = sum of min number of blocks obstructing all the misplaced blocks, 0 if m=0
         Pi = OnWorkspace(best_misplaced_objects), false if m=0
         Pl = OnWorkspace(placement : if placing attached object m ==)
  Rules:

  !H, m=0 -> END              --- END
  !H !Pi, m>0 -> !H, Pi       --- MOVE_TO_PICK_TABLE
  !H, Pi, m>0, n=0 -> H, m<   --- PICK_MISPLACED_OBJECT
  !H, Pi, m>0, n>0 -> H, s<   --- PICK_OBSTRUCTING_OBJECT
  H, !Pl -> H, Pl, m=, n=, s= --- MOVE_TO_PLACE_TABLE
  H, Pl-> !H, m=, n=, s=      --- PLACE_OBJECT

  // !H, m=0 -> END          --- END
  // !H, m>0, n=0 -> H, m<   --- PICK_MISPLACED_OBJECT
  // !H, m>0, n>0 -> H, s<   --- PICK_OBSTRUCTING_OBJECT
  // H -> !H, m=, n=, s=     --- PLACE_OBJECT
  */

  // TODO optimize computing s or not
  // std::cout<<"selecting sketch"<<std::endl;
  auto cloned_state = state->Clone();
  ComputeStateSketchFeatures(cloned_state);
  auto casted_state = dynamic_cast<const MoveitTampState *>(cloned_state);
  start_state_sketch_features_.H = casted_state->HasObjectAttached();
  start_state_sketch_features_.m = casted_state->GetNumOfMisplacedObjects();
  start_state_sketch_features_.n = casted_state->GetMinObstructingObjects();
  start_state_sketch_features_.s = casted_state->GetSumMinObjectsObstructingMisplacedObjects();
  if (start_state_sketch_features_.H) {
    start_state_sketch_features_.F = !AllGoalRegionBlockMisplaced(casted_state->GetAttatchedObject(), casted_state);
  } else {
    start_state_sketch_features_.F = false;
  }

  // TODO: should only be in the monotonic case
  if (block_misplaced_in_goal_ && !goal_positions_.empty()) {
    std::cout << "checking objects already in goal" << std::endl;
    const auto object_poses = casted_state->GetObjectPoses();
    for (auto const &goal : goal_positions_) {
      const std::string goal_id = goal.first.substr(goal.first.find("_") + 1);
      std::cout << "Checking id: " << goal_id << std::endl;
      for (const auto indice : object_indices) {
        if (indice.first.find(goal_id) != std::string::npos) {
          std::cout << "Checking object " << indice.first << std::endl;
          auto object = objects_.find(indice.first);
          if (object != objects_.end() && object->second.moveable_ &&
              OnCircle(object_poses.at(indice.second).translation(), goal_tolerance_radius_, goal.second)) {
            object->second.moveable_ = false;
            std::cout << "Object " << object->first << " is in goal position and is no longer moveable" << std::endl;
          }
        }
      }
      // target_blue
      // stick_blue1
    }
  }
  delete cloned_state;

  std::cout << "Selecting subproblem sketch: "
            << "\nStart state features are:"
            << " H: " << start_state_sketch_features_.H << " m: " << start_state_sketch_features_.m
            << " n: " << start_state_sketch_features_.n << " s: " << start_state_sketch_features_.s
            << " F: " << start_state_sketch_features_.F << "\nThe selected sketch is: " << std::endl;

  std::string x;
  std::cout << "STARTING SUBPROBLEM, type any key to continue" << std::endl;
  std::cin >> x;

  if (!start_state_sketch_features_.H) {
    if (start_state_sketch_features_.m == 0) {
      std::cout << "END PROBLEM" << std::endl;
      active_sketch_rule_ = MoveitTampProblem::END;
      return true;
    } else if (start_state_sketch_features_.m > 0) {
      if (start_state_sketch_features_.n == 0) {
        std::cout << "PICK_MISPLACED_OBJECT" << std::endl;

        active_sketch_rule_ = MoveitTampProblem::PICK_MISPLACED_OBJECT;
      } else if (start_state_sketch_features_.n > 0) {
        std::cout << "PICK_OBSTRUCTING_OBJECT" << std::endl;

        active_sketch_rule_ = MoveitTampProblem::PICK_OBSTRUCTING_OBJECT;
      } else {
        throw std::runtime_error("Malformed sketch feature min obstructing object");
      }
    } else {
      throw std::runtime_error("Malformed sketch feature num misplaced objects");
    }
  } else {
    // // TODO: FOR DEBUG ONLY REMOVE!
    // throw std::runtime_error("Stoped for debugging");
    if (start_state_sketch_features_.F) {
      std::cout << "PLACE_OBJECT_FREE" << std::endl;
      active_sketch_rule_ = MoveitTampProblem::PLACE_OBJECT_FREE;
    } else {
      std::cout << "PLACE_OBJECT_BLOCKED" << std::endl;
      active_sketch_rule_ = MoveitTampProblem::PLACE_OBJECT_BLOCKED;
    }
  }
  return true;
}

void MoveitTampProblem::SetBlockingObjects(MoveitTampState *const state, bool compute_s) const {

  std::size_t min_num_blocking_objects = std::numeric_limits<std::size_t>::max();
  std::size_t sum_min_blocking_objects = 0;
  std::cout << "Compute s is: " << compute_s << std::endl;
  for (const auto misplaced_object_index : state->GetMisplacedObjects()) {
    if (!compute_s && min_num_blocking_objects == 0)
      break;
    std::cout << "Setting blocking objects of misplaced object: " << object_names_.at(misplaced_object_index)
              << std::endl;
    const auto &misplaced_object = objects_.at(object_names_.at(misplaced_object_index));

    if (!goal_region_ && state->GetAttatchedObject() == misplaced_object.name_) {
      std::cout<<"Checking blocking objects of attached object. Min block obj is: "<<min_num_blocking_objects<<std::endl;
      auto target_pose =
          goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
      if (target_pose == goal_positions_.end())
        continue;
      std::size_t num_blocking_objects =
          BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), *state->GetGrasp(),
                               misplaced_object.stable_object_poses_, misplaced_object.name_,
                               compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects);
      if (num_blocking_objects < min_num_blocking_objects) {
        std::size_t blocked_by_goal = NumMisplacedsBlockedByGoal(
            Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), misplaced_object.name_, state);
        if (num_blocking_objects + blocked_by_goal < min_num_blocking_objects)
          min_num_blocking_objects = num_blocking_objects + blocked_by_goal;
      }
      if (compute_s)
        sum_min_blocking_objects += num_blocking_objects;
    } else {

      const auto object_pose = state->GetObjectPoses().at(misplaced_object_index);
      // std::cout << "Checking misplaced obj: " << misplaced_object.name_ << std::endl;

      std::size_t min_num_object_blocking_objects =
          compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects;
      std::size_t blocked_by_goal_best = 0;
      if (!goal_region_) {
        std::cout << "No goal region" << std::endl;
        auto target_pose =
            goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
        if (target_pose == goal_positions_.end())
          continue;
        std::size_t g_i = 0; // TODO borrar nomes debug
        for (const auto &grasp : misplaced_object.grasps_) {

          if (min_num_object_blocking_objects == 0)
            break;

          std::size_t num_blocking_objects = BlockingObjectsPick(
              state, object_pose, object_pose * grasp, misplaced_object.name_, min_num_object_blocking_objects);

          std::cout << "In grasp " << g_i << " ,num blocking objects pick are: " << num_blocking_objects << std::endl;

          if (min_num_object_blocking_objects > num_blocking_objects) {
            std::cout << "In grasp " << g_i << " ,num blocking have improved from " << min_num_object_blocking_objects
                      << " to " << num_blocking_objects << std::endl;
            num_blocking_objects +=
                BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), grasp,
                                     misplaced_object.stable_object_poses_, misplaced_object.name_,
                                     min_num_object_blocking_objects - num_blocking_objects);
            std::cout << "In grasp " << g_i << " , adding place, num blocking objects are: " << num_blocking_objects
                      << std::endl;
          }

          if (num_blocking_objects < min_num_object_blocking_objects) {
            std::cout << "In grasp " << g_i << " , includng place, num blocking have improved from "
                      << min_num_object_blocking_objects << " to " << num_blocking_objects << std::endl;
            std::size_t blocked_by_goal = NumMisplacedsBlockedByGoal(
                Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), misplaced_object.name_, state);
            std::cout << "In grasp " << g_i << " ,num blocked by goal is: " << blocked_by_goal << std::endl;
            if (num_blocking_objects + blocked_by_goal < min_num_object_blocking_objects + blocked_by_goal_best) {
              std::cout << "In grasp " << g_i << " , includng goal, num blocking have improved from "
                        << min_num_object_blocking_objects << " to " << num_blocking_objects
                        << " and best blocked by goal now is " << blocked_by_goal << std::endl;
              min_num_object_blocking_objects = num_blocking_objects;
              blocked_by_goal_best = blocked_by_goal;
            }
          }
          g_i++;
        }
      } else {
        for (const auto &grasp : misplaced_object.grasps_) {
          if (min_num_object_blocking_objects == 0)
            break;

          std::size_t num_blocking_objects = BlockingObjectsPick(
              state, object_pose, object_pose * grasp, misplaced_object.name_, min_num_object_blocking_objects);

          if (num_blocking_objects < min_num_object_blocking_objects) {
            min_num_object_blocking_objects = num_blocking_objects;
          }
        }
      }

      if (min_num_object_blocking_objects < min_num_blocking_objects)
        min_num_blocking_objects = min_num_object_blocking_objects + blocked_by_goal_best;
      if (compute_s)
        sum_min_blocking_objects += min_num_object_blocking_objects;
    }
  }

  if (min_num_blocking_objects > objects_.size())
    min_num_blocking_objects = 0;

  state->SetMinObstructingObjects(min_num_blocking_objects);
  if (compute_s)
    state->SetSumMinObjectsObstructingMisplacedObjects(sum_min_blocking_objects);
}

std::size_t MoveitTampProblem::BlockingObjectsPlace(MoveitTampState const *const state,
                                                    const Eigen::Affine3d &placement_pose, const Eigen::Affine3d &grasp,
                                                    const std::vector<Eigen::Affine3d> &sops,
                                                    const std::string &misplaced_object_name,
                                                    const std::size_t max_num_blocking_objects) const {
  // std::size_t min_num_blocking_objects = max_num_blocking_objects;
  std::size_t min_num_blocking_objects = 100;

  std::vector<Eigen::Affine3d> extra_dummy_poses;
  std::vector<std::string> blocking_objects_names_stored;
  std::cout << "Processing blocking place of object " << misplaced_object_name << std::endl;

  // // if object name contains red; see if green object is in front of him then change radius
  // double original_amplitude = gripper_semiamplitude_;
  // auto pos = misplaced_object_name.find("red");

  // if (pos != std::string::npos) {
  //   auto index = object_indices.find("stick_green" + misplaced_object_name.substr(pos + 3));
  //   if (index != object_indices.end()) {
  //     auto pose = state->GetObjectPoses().at(index->second);
  //     if(placement_pose.translation().x()+0.06 == pose.translation().x()){
  //       ROS_INFO("GREEN IS HERE");
  //       gripper_semiamplitude_=0.045;
  //     }else{
  //       ROS_INFO("MISSING GREEN");
  //     }
  //     /* code */
  //   }else{
  //     ROS_ERROR("MALFORMED NAME , INDEX NOT FOUND ");
  //   }
  // }
  for (const auto &robot_pose : base_locations_) {
    if (min_num_blocking_objects == 0)
      break;

    if (!OnWorkspace(robot_pose, placement_pose))
      continue;
    std::cout << "Checking robot pose: " << robot_pose.translation().transpose() << std::endl;
    for (const auto &sop : sops) {
      if (min_num_blocking_objects == 0) {
        break;
      }
      std::vector<std::string> blocking_objects_names;

      std::size_t num_blocking_objects =
          BlockingObjects(state, placement_pose * sop, robot_pose, placement_pose * sop * grasp, misplaced_object_name,
                          min_num_blocking_objects, blocking_objects_names, extra_dummy_poses);

      // auto pos = misplaced_object_name.find("red");
      // if (pos != std::string::npos) {
      //   geometry_msgs::Pose pose_msg;
      //   tf::poseEigenToMsg(placement_pose * sop, pose_msg);
      //   auto extra = display_placements_;
      //   extra.poses.push_back(pose_msg);
      //   pub_placements_.publish(extra);
      //   std::string x;

      //   std::cout << "type any key to continue" << std::endl;
      //   std::cin >> x;
      // }

      // if (num_blocking_objects < min_num_blocking_objects) {
      //   num_blocking_objects += NumMisplacedsBlockedByGoal(placement_pose, misplaced_object_name, state);
      //   if (num_blocking_objects < min_num_blocking_objects)
      //     min_num_blocking_objects = num_blocking_objects;
      // }
      if (num_blocking_objects < min_num_blocking_objects)
        min_num_blocking_objects = num_blocking_objects;
      blocking_objects_names_stored = blocking_objects_names;
    }
  }
  if (min_num_blocking_objects > 0)
    std::cout << "Object " << misplaced_object_name << " has the placement blocked by: " << std::endl;
  for (const auto &name : blocking_objects_names_stored) {
    std::cout << name << " , ";
  }
  std::cout << std::endl;
  return min_num_blocking_objects;
}

std::size_t MoveitTampProblem::BlockingObjectsPick(MoveitTampState const *const state,
                                                   const Eigen::Affine3d &object_pose,
                                                   const Eigen::Affine3d &gripper_pose,
                                                   const std::string &misplaced_object_name,
                                                   const std::size_t max_num_blocking_objects) const {
  std::size_t min_num_blocking_objects = max_num_blocking_objects;
  std::vector<Eigen::Affine3d> extra_dummy_poses;

  for (const auto &robot_pose : base_locations_) {
    if (min_num_blocking_objects == 0)
      break;

    if (!OnWorkspace(robot_pose, object_pose))
      continue;
    std::vector<std::string> blocking_objects_names;
    std::size_t num_blocking_objects =
        BlockingObjects(state, object_pose, robot_pose, gripper_pose, misplaced_object_name, min_num_blocking_objects,
                        blocking_objects_names, extra_dummy_poses);

    if (num_blocking_objects < min_num_blocking_objects)
      min_num_blocking_objects = num_blocking_objects;
  }

  return min_num_blocking_objects;
}

std::size_t MoveitTampProblem::BlockingObjects(MoveitTampState const *const state, const Eigen::Affine3d &target_pose,
                                               const Eigen::Affine3d &robot_pose, const Eigen::Affine3d &gripper_pose,
                                               const std::string &misplaced_object_name,
                                               const std::size_t max_num_blocking_objects,
                                               std::vector<std::string> &blocking_objects_names,
                                               std::vector<Eigen::Affine3d> &extra_poses,
                                               const bool reduced_amplitude) const {
  Eigen::Vector3d g = gripper_pose.translation();
  Eigen::Vector3d r = robot_pose.translation() - g;
  r(2) = 0;
  Eigen::Vector3d o = target_pose.translation() - g;
  o(2) = 0;

  std::size_t num_blocking_objects = 0;
  for (const auto &obstructing_object : objects_) {
    if (num_blocking_objects >= max_num_blocking_objects)
      break;

    if (!obstructing_object.second.moveable_ || state->GetAttatchedObject() == obstructing_object.second.name_ ||
        obstructing_object.second.name_ == misplaced_object_name)
      continue;

    Eigen::Vector3d w =
        state->GetObjectPoses().at(object_indices.at(obstructing_object.second.name_)).translation() - g;
    w(2) = 0;

    double dist = (w - std::min(std::max(0.0, w.dot(r) / r.squaredNorm()), 1.0) * r).norm();

    if (dist < blocking_object_distance_threshold_) {
      num_blocking_objects++;
      blocking_objects_names.push_back(obstructing_object.second.name_);
      if (misplaced_object_name.find("red") != std::string::npos) {
        // std::cout
        //     << "IN LONG DISTANCE, For misplaced object " << misplaced_object_name << " object "
        //     << obstructing_object.second.name_ << " is blocking. Poses are: " <<
        //     target_pose.translation().transpose()
        //     << " and "
        //     <<
        //     state->GetObjectPoses().at(object_indices.at(obstructing_object.second.name_)).translation().transpose()
        //     << " Distance is: " << dist << std::endl;
      }

    } else {
      double proj = w.dot(o) / o.squaredNorm();
      if (proj <= 1 + 0.04 / o.norm()) {
        dist = (w - std::max(0.0, proj) * o).norm();
        if (dist < blocking_object_distance_threshold_) {
          num_blocking_objects++;
          blocking_objects_names.push_back(obstructing_object.second.name_);
          if (misplaced_object_name.find("red") != std::string::npos) {
            // std::cout << "IN SHORT DISTANCE, For misplaced object " << misplaced_object_name << " object "
            //           << obstructing_object.second.name_
            //           << " is blocking. Poses are: " << target_pose.translation().transpose() << " and "
            //           << state->GetObjectPoses()
            //                  .at(object_indices.at(obstructing_object.second.name_))
            //                  .translation()
            //                  .transpose()
            //           << " Distance is: " << dist << std::endl;
          }
        }
      }
      // dist = (w - std::min(std::max(0.0, w.dot(o) / o.squaredNorm()), 1.0) * o).norm();
      // if (dist < blocking_object_distance_threshold_) {
      //   num_blocking_objects++;
      //   blocking_objects_names.push_back(obstructing_object.second.name_);
      // }
    }
  }
  for (const auto &pose : extra_poses) {
    if (num_blocking_objects >= max_num_blocking_objects)
      break;
    Eigen::Vector3d w = pose.translation() - g;
    w(2) = 0;

    double dist = (w - std::min(std::max(0.0, w.dot(r) / r.squaredNorm()), 1.0) * r).norm();
    // std::cout << "Checking blocking goal pose " << pose.translation().transpose()
    //           << " and misplaced object pose is: " << target_pose.translation().transpose() << " distance is: " <<
    //           dist
    //           << std::endl;

    if (dist < blocking_object_distance_threshold_) {
      num_blocking_objects++;
      blocking_objects_names.push_back("goal");
      // std::cout << "Inserting blocking goal pose " << pose.translation().transpose() << " distance is: " << dist
      //           << std::endl;

    } else {

      double proj = std::max(0.0, w.dot(o) / o.squaredNorm());
      if (proj <= 1 + 0.04 / o.norm()) {
        dist = (w - proj * o).norm();
        if (dist < blocking_object_distance_threshold_) {
          num_blocking_objects++;
          blocking_objects_names.push_back("goal");
          // std::cout << "Inserting blocking goal pose " << pose.translation().transpose() << " distance is: " << dist
          //           << std::endl;
        }
      }

      // dist = (w - std::min(std::max(0.0, w.dot(o) / o.squaredNorm()), 1.0) * o).norm();
      // //  std::cout << "Checking blocking goal pose " << pose.translation().transpose()
      // //            << " and misplaced object pose is: " << target_pose.translation().transpose()
      // //            << " distance is: " << dist << std::endl;
      // if (dist < blocking_object_distance_threshold_) {
      //   num_blocking_objects++;
      //   blocking_objects_names.push_back("goal");
      //   std::cout << "Inserting blocking goal pose " << pose.translation().transpose() << " distance is: " << dist
      //           << std::endl;
      // }
    }
  }

  return num_blocking_objects;
}

bool MoveitTampProblem::AllGoalRegionBlockMisplaced(const std::string &object_name,
                                                    MoveitTampState const *const state) const {
  // Get sampled goal poses for the object of interest
  std::vector<Eigen::Affine3d> sampled_goal_poses;
  for (const auto &object : objects_) {
    if (object.first == object_name)
      continue;
    for (const auto &surface : object.second.surfaces_) {
      for (const auto &placement : surface.placements_) {
        if (!Misplaced(object_name, object.second.pose_ * placement)) {
          sampled_goal_poses.push_back(object.second.pose_ * placement);
          std::cout << "Sampled goal pose for object " << object_name << " : "
                    << (object.second.pose_ * placement).translation().transpose() << std::endl;
        }
      }
    }
  }
  if (sampled_goal_poses.size() == 0) {
    std::cout << "This object doesnt have goal targets" << std::endl;
    return true;
  }
  // Check if at least one goal poses does not block any misplaced object
  // const auto grasped_object = state->GetAttatchedObject();
  for (const auto goal_pose : sampled_goal_poses) {
    if (NumMisplacedsBlockedByGoal(goal_pose, object_name, state, true) == 0) {
      return false; // RETURN THAT AT LEAST ONE GOAL POSE IS FREE
    }
  }
  return true; // RETURN THAT ALL GOAL POSE BLOCK AT LEAST ONE MISPLACED
}
std::size_t MoveitTampProblem::NumMisplacedsBlockedByGoal(const Eigen::Affine3d &goal_pose,
                                                          const std::string &object_name,
                                                          MoveitTampState const *const state,
                                                          const bool only_check_at_least_one_is_blocked) const {

  // Iterate Misplaced objects
  std::vector<Eigen::Affine3d> goal_poses;
  std::size_t num_misplaced_blocked_by_goal = 0;
  goal_poses.push_back(goal_pose);
  for (const auto misplaced_object_index : state->GetMisplacedObjects()) {
    const auto &misplaced_object = objects_.at(object_names_.at(misplaced_object_index));
    if (object_name == misplaced_object.name_)
      continue;
    std::size_t min_num_blocking_objects = std::numeric_limits<std::size_t>::max();
    bool goal_is_blocking = false;

    const auto object_pose = state->GetObjectPoses().at(misplaced_object_index);
    // if (!OnWorkspace(
    //         goal_pose,
    //         object_pose)) // TODO: Pendent de fer mes elegant. Comprovar que els dos objectes estan minimament aprop
    //   continue;
    // Iterate misplaced grasps
    Eigen::Affine3d blocked_pose;
    std::string blocked_operation;
    std::cout << "Checking if object " << object_name << " blocks misplaced: " << misplaced_object.name_ << std::endl;
    for (const auto &grasp : misplaced_object.grasps_) {
      const auto gripper_pose = object_pose * grasp;
      // if (min_num_object_blocking_objects == 0)
      if (min_num_blocking_objects == 0)
        break;

      // Iterate Base Location (check if on WS)
      for (const auto &robot_pose : base_locations_) {
        if (min_num_blocking_objects == 0)
          break;

        if (!OnWorkspace(robot_pose, object_pose))
          continue;
        std::vector<std::string> blocking_objects_names;
        std::size_t num_blocking_objects =
            BlockingObjects(state, object_pose, robot_pose, gripper_pose, misplaced_object.name_,
                            min_num_blocking_objects, blocking_objects_names, goal_poses);
        std::cout << "With robot pose " << robot_pose.translation().transpose()
                  << "\n gripper_pose: " << gripper_pose.translation().transpose()
                  << "\n min num: " << min_num_blocking_objects
                  << " \n the number of blocking in PICK is: " << num_blocking_objects << "\n and blocking goal is: "
                  << (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                      blocking_objects_names.end())
                  << std::endl;

        if (num_blocking_objects < min_num_blocking_objects) {
          if (!goal_region_) {
            auto target_pose =
                goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
            if (target_pose == goal_positions_.end())
              continue;

            for (const auto &sop : misplaced_object.stable_object_poses_) {
              const auto &placement_pose = Eigen::Affine3d(Eigen::Translation3d(target_pose->second));
              num_blocking_objects +=
                  BlockingObjects(state, placement_pose * sop, robot_pose, placement_pose * sop * grasp,
                                  misplaced_object.name_, min_num_blocking_objects, blocking_objects_names, goal_poses);
              if (num_blocking_objects < min_num_blocking_objects) {
                min_num_blocking_objects = num_blocking_objects;
                goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                    blocking_objects_names.end());
                // if (goal_is_blocking) {
                //   blocked_pose = placement_pose;          
                // }
                std::cout << "With sop " << sop.rotation()                        
                          << "\n min num: " << min_num_blocking_objects
                          << " \n the number of blocking in PLACE is: " << num_blocking_objects
                          << "\n and blocking goal is: "
                          << (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                              blocking_objects_names.end())
                          << std::endl;
              } else if (num_blocking_objects == min_num_blocking_objects && goal_is_blocking) {
                goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                    blocking_objects_names.end());
                std::cout << "With sop " << sop.rotation() << "\n min num: " << min_num_blocking_objects
                          << " \n the number of blocking in PLACE is: " << num_blocking_objects
                          << "\n and blocking goal is: "
                          << (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                              blocking_objects_names.end())
                          << std::endl;
              }
              if (min_num_blocking_objects == 0)
                break;
            }
          } else {
            min_num_blocking_objects = num_blocking_objects;
            goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                blocking_objects_names.end());
          }

        } else if (num_blocking_objects == min_num_blocking_objects && goal_is_blocking) {
          if (!goal_region_) {
            auto target_pose =
                goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
            if (target_pose == goal_positions_.end())
              continue;

            for (const auto &sop : misplaced_object.stable_object_poses_) {
              const auto &placement_pose = Eigen::Affine3d(Eigen::Translation3d(target_pose->second));
              num_blocking_objects +=
                  BlockingObjects(state, placement_pose * sop, robot_pose, placement_pose * sop * grasp,
                                  misplaced_object.name_, min_num_blocking_objects, blocking_objects_names, goal_poses);
              if (num_blocking_objects < min_num_blocking_objects) {
                min_num_blocking_objects = num_blocking_objects;
                goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                    blocking_objects_names.end());
              }

              else if (num_blocking_objects == min_num_blocking_objects && goal_is_blocking) {
                goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                    blocking_objects_names.end());
              }
              if (min_num_blocking_objects == 0)
                break;
            }

          } else {
            goal_is_blocking = (std::find(blocking_objects_names.begin(), blocking_objects_names.end(), "goal") !=
                                blocking_objects_names.end());
          }
        }
      }
    }
    if (goal_is_blocking) {
      num_misplaced_blocked_by_goal++;
      auto target_pose =
          goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
      std::cout << "Object " << object_name << " goal pose: " << goal_pose.translation().transpose()
                << " blocks misplaced " << misplaced_object.name_
                << " with pick pose: " << object_pose.translation().transpose()

                << " and placement pose: "
                << Eigen::Affine3d(Eigen::Translation3d(target_pose->second)).translation().transpose() << std::endl;
      std::string x;

      std::cout << "type any key to continue" << std::endl;
      std::cin >> x;

      if (only_check_at_least_one_is_blocked) {
        std::cout << "Number of misplaced blocked by goal is: " << num_misplaced_blocked_by_goal << std::endl;
        return num_misplaced_blocked_by_goal;
      }
    }
  }
  std::cout << "Number of misplaced blocked by goal is: " << num_misplaced_blocked_by_goal << std::endl;
  return num_misplaced_blocked_by_goal;
}

void MoveitTampProblem::PrintStatistics() const {
  std::cout << "Move: " << num_move_base_ << " Pick: " << num_pick_ << " Place: " << num_place_
            << " Num total actions: " << num_move_base_ + num_pick_ + num_place_ + reused_valid_actions_
            << " Num reused actions: " << reused_valid_actions_ << "\nTotal IK calls: " << num_total_ik_calls_
            << " Successful IK calls: " << num_successful_ik_calls_
            << "\nTotal motion plans: " << num_total_motion_plans_
            << " Successful motion plans: " << num_successful_motion_plans_ << std::endl;
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
                                                        const Eigen::Affine3d &surface_pose, double discretization,
                                                        double padding)
    : pose_(surface_pose) {
  min_(0) = x_min;
  min_(1) = y_min;
  max_(0) = x_max;
  max_(1) = y_max;
  discretization_ = discretization;
  padding_ = padding;
};

double MoveitTampProblem::SupportingSurface::area() const { return (max_ - min_).prod(); }
bool MoveitTampProblem::SupportingSurface::on(const Eigen::Vector3d &position) const {
  const auto v = pose_.inverse() * position;
  const double tol = 1e-6;
  return (v(0) >= min_(0) - tol && v(0) <= max_(0) + tol) && (v(1) >= min_(1) - tol && v(1) <= max_(1) + tol) &&
         (v(2) >= 0. - tol && v(2) <= 0. + tol);
}

// void MoveitTampProblem::SupportingSurface::sample(std::size_t num_samples) {
//   const unsigned int num_bins = std::max(1., std::ceil(sqrt(double(placements_.size()))));
//   std::vector<double> intervals_x(num_bins + 1), intervals_y(num_bins + 1), intervals_yaw(num_bins + 1);
//   std::vector<std::size_t> count_x(num_bins, 0), count_y(num_bins, 0), count_yaw(num_bins, 0);
//   double x, y, yaw;
//   for (unsigned int i = 0; i <= num_bins; ++i) {
//     intervals_x.at(i) = 0.05 + min_(0) + (max_(0) - min_(0) - 2 * 0.05) * double(i) / double(num_bins);
//     intervals_y.at(i) = 0.05 + min_(1) + (max_(1) - min_(1) - 2 * 0.05) * double(i) / double(num_bins);
//     intervals_yaw.at(i) = -M_PI + 2. * M_PI * double(i) / double(num_bins);
//   }
//   for (const auto &placement : placements_) {
//     const auto local_pose = pose_.inverse() * placement;
//     count_x.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(0) - min_(0)) /
//                                                                (max_(0) - min_(0)) * double(num_bins))))++;
//     count_y.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(1) - min_(1)) /
//                                                                (max_(1) - min_(1)) * double(num_bins))))++;
//     count_yaw.at(
//         std::min(num_bins - 1, (unsigned int)std::floor((atan2(local_pose.matrix()(1, 0) - local_pose.matrix()(0,
//         1),
//                                                                local_pose.matrix()(0, 0) + local_pose.matrix()(1,
//                                                                1))
//                                                                +
//                                                          M_PI) /
//                                                         (2. * M_PI) * double(num_bins))))++;
//   }
//   std::size_t min_count_x = *std::min_element(count_x.begin(), count_x.end());
//   std::size_t min_count_y = *std::min_element(count_y.begin(), count_y.end());
//   std::size_t min_count_yaw = *std::min_element(count_yaw.begin(), count_yaw.end());
//   std::vector<double> weights_x(num_bins, 0), weights_y(num_bins, 0), weights_yaw(num_bins, 0);
//   for (std::size_t i = 0; i < num_bins; ++i) {
//     if (count_x.at(i) == min_count_x)
//       weights_x.at(i) = 1;
//     if (count_y.at(i) == min_count_y)
//       weights_y.at(i) = 1;
//     if (count_yaw.at(i) == min_count_yaw)
//       weights_yaw.at(i) = 1;
//   }
//   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//   std::default_random_engine generator(seed);
//   std::piecewise_constant_distribution<double> distribution_x(intervals_x.begin(), intervals_x.end(),
//                                                               weights_x.begin());
//   std::piecewise_constant_distribution<double> distribution_y(intervals_y.begin(), intervals_y.end(),
//                                                               weights_y.begin());
//   std::piecewise_constant_distribution<double> distribution_yaw(intervals_yaw.begin(), intervals_yaw.end(),
//                                                                 weights_yaw.begin());
//   placements_.push_back(pose_ * Eigen::Translation3d(distribution_x(generator), distribution_y(generator), 0) *
//                         Eigen::AngleAxisd(distribution_yaw(generator), Eigen::Vector3d::UnitZ()));

//   if (num_samples > 1)
//     sample(num_samples - 1);
// }

void MoveitTampProblem::SupportingSurface::sample(std::size_t num_samples) {
  const std::size_t num_bins_x = std::max(1., std::ceil((max_(0) - min_(0) - 2 * padding_) / discretization_));
  const std::size_t num_bins_y = std::max(1., std::ceil((max_(1) - min_(1) - 2 * padding_) / discretization_));
  const std::size_t num_bins_yaw = 4;

  std::vector<double> intervals_x(num_bins_x + 1), intervals_y(num_bins_y + 1), intervals_yaw(num_bins_yaw + 1);
  for (std::size_t k = 0; k <= num_bins_x; ++k) {
    intervals_x.at(k) = padding_ + min_(0) + (max_(0) - min_(0) - 2 * padding_) * double(k) / double(num_bins_x);
  }
  for (std::size_t k = 0; k <= num_bins_y; ++k) {
    intervals_y.at(k) = padding_ + min_(1) + (max_(1) - min_(1) - 2 * padding_) * double(k) / double(num_bins_y);
  }
  for (std::size_t k = 0; k <= num_bins_yaw; ++k) {
    intervals_yaw.at(k) = -M_PI + 2. * M_PI * double(k) / double(num_bins_yaw);
  }
  std::vector<std::size_t> count_xy(num_bins_x * num_bins_y, 0), count_yaw(num_bins_yaw, 0);
  for (const auto &placement : placements_) {
    const auto local_pose = pose_.inverse() * placement;
    std::size_t index_x =
        std::min(num_bins_x - 1, (std::size_t)std::floor((local_pose.translation()(0) - min_(0) - padding_) /
                                                         (max_(0) - min_(0) - 2 * padding_) * double(num_bins_x)));
    std::size_t index_y =
        std::min(num_bins_y - 1, (std::size_t)std::floor((local_pose.translation()(1) - min_(1) - padding_) /
                                                         (max_(1) - min_(1) - 2 * padding_) * double(num_bins_y)));
    std::size_t index_yaw = std::min(
        num_bins_yaw - 1, (std::size_t)std::floor((atan2(local_pose.matrix()(1, 0) - local_pose.matrix()(0, 1),
                                                         local_pose.matrix()(0, 0) + local_pose.matrix()(1, 1)) +
                                                   M_PI) /
                                                  (2. * M_PI) * double(num_bins_yaw)));
    count_xy.at(index_x * num_bins_y + index_y)++;
    count_yaw.at(index_yaw)++;
  }
  std::size_t min_count_xy = *std::min_element(count_xy.begin(), count_xy.end());
  std::size_t min_count_yaw = *std::min_element(count_yaw.begin(), count_yaw.end());
  std::vector<double> weights_xy(num_bins_x * num_bins_y, 0), weights_yaw(num_bins_yaw, 0);
  for (std::size_t k = 0; k < num_bins_x * num_bins_y; ++k) {
    if (count_xy.at(k) == min_count_xy)
      weights_xy.at(k) = 1;
  }
  for (std::size_t k = 0; k < num_bins_yaw; ++k) {
    if (count_yaw.at(k) == min_count_yaw)
      weights_yaw.at(k) = 1;
  }
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::discrete_distribution<std::size_t> distribution_xy(weights_xy.begin(), weights_xy.end());
  const std::size_t index_xy = distribution_xy(generator);
  const std::size_t index_x = index_xy / num_bins_y;
  const std::size_t index_y = index_xy % num_bins_y;
  std::uniform_real_distribution<double> distribution_x(intervals_x.at(index_x), intervals_x.at(index_x + 1));
  std::uniform_real_distribution<double> distribution_y(intervals_y.at(index_y), intervals_y.at(index_y + 1));
  std::piecewise_constant_distribution<double> distribution_yaw(intervals_yaw.begin(), intervals_yaw.end(),
                                                                weights_yaw.begin());
  placements_.push_back(pose_ * Eigen::Translation3d(distribution_x(generator), distribution_y(generator), 0) *
                        Eigen::AngleAxisd(distribution_yaw(generator), Eigen::Vector3d::UnitZ()));

  if (num_samples > 1)
    sample(num_samples - 1);
  else {
    // std::cout << "Sampled placements grid:" << std::endl;
    // for (std::size_t i = 0; i < num_bins_x; ++i) {
    //   for (std::size_t j = 0; j < num_bins_y; ++j) {
    //     std::cout << count_xy.at(i * num_bins_y + j);
    //     if (j + 1 < num_bins_y)
    //       std::cout << " ";
    //     else
    //       std::cout << std::endl;
    //   }
    // }
    // std::cout << std::endl;
  }
}

void MoveitTampProblem::SupportingSurface::sample() {
  const unsigned int num_bins = std::max(1., std::ceil(sqrt(double(placements_.size()))));
  std::vector<double> weights_x(num_bins, 0), weights_y(num_bins, 0), weights_yaw(num_bins, 0);
  std::vector<double> intervals_x(num_bins + 1), intervals_y(num_bins + 1), intervals_yaw(num_bins + 1);
  double x, y, yaw;
  for (unsigned int i = 0; i <= num_bins; ++i) {
    intervals_x.at(i) = 0.025 + min_(0) + (max_(0) - min_(0) - 2 * 0.025) * double(i) / double(num_bins);
    intervals_y.at(i) = 0.025 + min_(1) + (max_(1) - min_(1) - 2 * 0.025) * double(i) / double(num_bins);
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