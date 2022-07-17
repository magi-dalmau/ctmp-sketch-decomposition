#include <non_monotonic_tamp_problem.hpp>

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

NonMonotonicTampProblem::NonMonotonicTampProblem(const std::string &filename, const std::string &planning_group,
                                                 ros::NodeHandle *nodehandle)
    : MoveitTampProblem(filename, planning_group, nodehandle) {
  goal_tolerance_radius_ = nh_.param("goal_tolerance_radius_", 0.005);
  // Load world
  
  LoadWorld(filename);
std::cout<<"NUM OF TARGET GOALS IS: "<<goal_positions_.size()<<std::endl;
  ROS_DEBUG("PROBLEM INITIALIZED");
}

// void NonMonotonicTampProblem::LoadWorld(const std::string &filename) {

//   double x_distance_arm_tool_link_to_grasping_point = nh_.param("x_dist_arm_tool_link_to_grasping_point", 0.20);

//   std::size_t num_grasps = nh_.param("num_grasps", 4);
//   std::size_t num_sops = nh_.param("num_sops", 1);

//   Eigen::Affine3d arm_tool_link_to_grasp_point_inv =
//       (Eigen::Affine3d().fromPositionOrientationScale(Eigen::Vector3d(x_distance_arm_tool_link_to_grasping_point, 0, 0),
//                                                       Eigen::Quaterniond(0.7071068, 0.7071068, 0, 0),
//                                                       Eigen::Vector3d(1, 1, 1)))
//           .inverse();

//   std::map<std::string, shape_msgs::Mesh> meshes_map;
//   std::vector<moveit_msgs::CollisionObject> vector_collision_objects;
//   moveit_msgs::CollisionObject collision_object;
//   collision_object.operation = moveit_msgs::CollisionObject::ADD;
//   collision_object.header.frame_id = common_reference_;
//   collision_object.meshes.resize(1);
//   geometry_msgs::Pose origin_mesh;
//   origin_mesh.orientation.x = 0;
//   origin_mesh.orientation.y = 0;
//   origin_mesh.orientation.z = 0;
//   origin_mesh.orientation.w = 1;
//   origin_mesh.position.x = 0;
//   origin_mesh.position.y = 0;
//   origin_mesh.position.z = 0;
//   collision_object.mesh_poses.push_back(origin_mesh);

//   TiXmlDocument doc(filename);
//   doc.LoadFile();
//   if (!doc.LoadFile()) {
//     ROS_ERROR("Failed to load file [%s] with error %s.", filename.c_str(), doc.ErrorDesc());
//     throw std::runtime_error("Failed to load file [" + filename + "] with error " + doc.ErrorDesc() + ".");
//   }

//   // Lambda function string ('true' or '1') to bool
//   auto string2bool = [](const std::string &str) {
//     return boost::algorithm::to_lower_copy(str).compare("true") == 0 || str.compare("1") == 0;
//   };
//   auto string2pose = [](const std::string &str) {
//     Eigen::Affine3d pose;
//     std::vector<std::string> values;
//     boost::split(values, str, boost::is_any_of(" "));
//     for (unsigned int i = 0; i < 3; ++i)
//       for (unsigned int j = 0; j < 4; ++j)
//         pose.matrix()(i, j) = std::stod(values.at(i * 4 + j));

//     return pose;
//   };

//   auto string2rot = [](const std::string &str) {
//     Eigen::Matrix3d rotation_matrix;
//     std::vector<std::string> values;
//     boost::split(values, str, boost::is_any_of(" "));
//     for (unsigned int i = 0; i < 3; ++i)
//       for (unsigned int j = 0; j < 3; ++j)
//         rotation_matrix(i, j) = std::stod(values.at(i * 3 + j));

//     return rotation_matrix;
//   };

//   auto string2vector = [](const std::string &str) {
//     Eigen::Vector3d vector;
//     std::vector<std::string> values;
//     boost::split(values, str, boost::is_any_of(" "));
//     for (unsigned int i = 0; i < 3; ++i)
//       vector(i) = std::stod(values.at(i));

//     return vector;
//   };

//   unsigned int num_surfaces = 0;
//   TiXmlHandle h_doc(&doc);

//   // PARSE ROBOTS
//   // TODO:#FUTURE IMPROVEMENT: Only one robot considered, multirobot pending
//   auto rob = h_doc.FirstChildElement("problem").FirstChildElement("robots").FirstChildElement("robot").ToElement();
//   robot_origin_ = string2pose(rob->FirstChildElement("basepose")->GetText());
//   robot_pose_ = robot_origin_;
//   std::cout << "loaded robot origin" << std::endl;
//   // PARSE OBJECTS
//   for (auto obj = h_doc.FirstChildElement("problem").FirstChildElement("objects").FirstChildElement("obj").ToElement();
//        obj; obj = obj->NextSiblingElement("obj"))

//   {
//     Object object;
//     object.name_ = obj->FirstChildElement("name")->GetText();
//     object.pose_ = string2pose(obj->FirstChildElement("pose")->GetText());
//     std::cout << "Checking name: " << object.name_ << std::endl;
//     if (object.name_.find("target") != std::string::npos) {
//       goal_positions_.insert(std::make_pair(object.name_, object.pose_ * Eigen::Vector3d(0, 0, 0.001)));
//       continue;
//     }

//     object.mesh_ = obj->FirstChildElement("geom")->GetText();
//     object.moveable_ = string2bool(obj->FirstChildElement("moveable")->GetText());
//     double surface_offset = 0;
//     if (object.moveable_) {
//       object.pose_ = object.pose_ * Eigen::Translation3d(0, 0, 0.001);
//     } else {
//       surface_offset = 0.001;
//     }

//     for (auto sssp = obj->FirstChildElement("sssp"); sssp; sssp = sssp->NextSiblingElement("sssp")) {
//       const double x_min = std::stod(sssp->FirstChildElement("xmin")->GetText());
//       const double x_max = std::stod(sssp->FirstChildElement("xmax")->GetText());
//       const double y_min = std::stod(sssp->FirstChildElement("ymin")->GetText());
//       const double y_max = std::stod(sssp->FirstChildElement("ymax")->GetText());
//       const double z = std::stod(sssp->FirstChildElement("zmin")->GetText());
//       if (object.surfaces_.empty()) {
//         supporting_object_names.push_back(object.name_);
//       }
//       object.surfaces_.push_back(SupportingSurface(x_min, x_max, y_min, y_max,
//                                                    Eigen::Affine3d(Eigen::Translation3d(0, 0, z + surface_offset)),
//                                                    discretization_, padding_));

//       visualization_msgs::Marker display_surface;
//       display_surface.header.frame_id = common_reference_;
//       display_surface.ns = "surfaces";
//       display_surface.id = display_objects_.markers.size();
//       display_surface.type = visualization_msgs::Marker::TRIANGLE_LIST;
//       display_surface.action = visualization_msgs::Marker::ADD;
//       tf::poseEigenToMsg(object.pose_ * Eigen::Translation3d(0, 0, z), display_surface.pose);
//       display_surface.frame_locked = true;
//       display_surface.scale.x = 1;
//       display_surface.scale.y = 1;
//       display_surface.scale.z = 1;
//       display_surface.color.r = 1;
//       display_surface.color.g = 1;
//       display_surface.color.b = 0;
//       display_surface.color.a = 1;
//       geometry_msgs::Point point;
//       point.x = x_min;
//       point.y = y_min;
//       display_surface.points.push_back(point);
//       point.x = x_max;
//       display_surface.points.push_back(point);
//       point.y = y_max;
//       display_surface.points.push_back(point);
//       display_surface.points.push_back(point);
//       point.x = x_min;
//       display_surface.points.push_back(point);
//       point.y = y_min;
//       display_surface.points.push_back(point);
//       display_objects_.markers.push_back(display_surface);
//     }

//     num_surfaces += object.surfaces_.size();

//     // POPULATE GRASPS
//     auto grasps = obj->FirstChildElement("grasps");
//     if (grasps) {
//       for (auto grasp = grasps->FirstChildElement("gc"); grasp; grasp = grasp->NextSiblingElement("gc")) {
//         auto pose = string2pose(grasp->FirstChildElement("template")->GetText());
//         auto axis = string2vector(grasp->FirstChildElement("axis")->GetText());
//         for (unsigned int i = 0; i < num_grasps; ++i)
//           // TODO check rot*pos o pos*rot
//           object.grasps_.push_back((pose * Eigen::AngleAxisd((2.0 * M_PI / double(num_sops)) * i, axis)) *
//                                    arm_tool_link_to_grasp_point_inv);
//       }
//       for (auto grasp = grasps->FirstChildElement("gf"); grasp; grasp = grasp->NextSiblingElement("gf")) {
//         object.grasps_.push_back(string2pose(grasp->GetText()) * arm_tool_link_to_grasp_point_inv);
//       }
//     }

//     // POPULATE SOPS
//     for (auto sop = obj->FirstChildElement("sop"); sop; sop = sop->NextSiblingElement("sop")) {
//       auto pose = Eigen::Affine3d().fromPositionOrientationScale(
//           Eigen::Vector3d::Zero(), string2rot(sop->FirstChildElement("template")->GetText()), Eigen::Vector3d::Ones());
//       auto axis = string2vector(sop->FirstChildElement("axis")->GetText());
//       for (unsigned int i = 0; i < num_sops; ++i)
//         // TODO check rot*pos o pos*rot
//         object.stable_object_poses_.push_back(pose * Eigen::AngleAxisd((2.0 * M_PI / double(num_sops)) * i, axis));
//     }

//     // POPULATE PLACEMENTS
//     if (obj->FirstChildElement("attachments")) {
//       for (auto attachment = obj->FirstChildElement("attachments")->FirstChildElement("name"); attachment;
//            attachment = attachment->NextSiblingElement("name")) {
//         auto &other_object = objects_.at(attachment->GetText());
//         for (auto &surface : object.surfaces_) {
//           if (surface.on(object.pose_.inverse() * other_object.pose_.translation()))
//             surface.placements_.push_back(object.pose_.inverse() * other_object.pose_);
//         }
//       }
//     }

//     objects_.insert(std::make_pair(object.name_, object));
//     object_names_.push_back(object.name_);
//     object_indices.insert(std::make_pair(object.name_, object_names_.size() - 1));
//     object_idx_to_marker_idx_.push_back(display_objects_.markers.size());
//     std::cout << "Inserted object " << object.name_ << std::endl;

//     visualization_msgs::Marker display_object;
//     display_object.header.frame_id = common_reference_;
//     display_object.ns = object.moveable_ ? "moveable_objects" : "static_objects";
//     display_object.id = display_objects_.markers.size();
//     display_object.type = visualization_msgs::Marker::MESH_RESOURCE;
//     std::string mesh_name = object.mesh_.substr(0, object.mesh_.find_last_of('.'));
//     display_object.mesh_resource =
//         "file://" + filename.substr(0, filename.substr(0, filename.find_last_of('/')).find_last_of('/')) + "/meshes/" +
//         mesh_name + ".dae";
//     // display_object.mesh_use_embedded_materials = true;
//     // TODO #FUTURE MINOR:(magi.dalmau) solve color problem definition in files
//     display_object.action = visualization_msgs::Marker::ADD;
//     tf::poseEigenToMsg(object.pose_, display_object.pose);
//     display_object.frame_locked = true;
//     display_object.scale.x = 1;
//     display_object.scale.y = 1;
//     display_object.scale.z = 1;
//     if (object.name_.find("red") != std::string::npos) {
//       display_object.color.r = 1;
//       display_object.color.g = 0;
//       display_object.color.b = 0;
//       display_object.mesh_use_embedded_materials = false;
//     } else if (object.name_.find("green") != std::string::npos) {
//       display_object.color.r = 0;
//       display_object.color.g = 1;
//       display_object.color.b = 0;
//       display_object.mesh_use_embedded_materials = false;
//     } else if (object.name_.find("blue") != std::string::npos) {
//       display_object.color.r = 0;
//       display_object.color.g = 0;
//       display_object.color.b = 1;
//       display_object.mesh_use_embedded_materials = false;
//     } else {
//       display_object.color.r = 0.5;
//       display_object.color.g = 0.5;
//       display_object.color.b = 0.5;
//       display_object.mesh_use_embedded_materials = false;
//     }
//     display_object.color.a = 1;
//     display_objects_.markers.push_back(display_object);

//     // moveit collision objects operations

//     // Set the particular values of the collision object
//     collision_object.id = object.name_;
//     collision_object.pose = display_object.pose;

//     // Build mesh msg if not done yet
//     const auto mesh_it = meshes_map.find(mesh_name);
//     if (mesh_it == meshes_map.end()) {
//       std::cout << "Loading mesh " << mesh_name << std::endl;
//       auto mesh = MeshMsgFromFile(display_object.mesh_resource);
//       meshes_map.insert(std::make_pair(mesh_name, mesh));
//       std::cout << "mesh lodaded" << std::endl;
//       collision_object.meshes.at(0) = mesh; // note that the mesh vector is already initialized in the common operations
//     } else {
//       collision_object.meshes.at(0) =
//           mesh_it->second; // note that the mesh vector is already initialized in the common operations
//     }

//     std::cout << "Inserted collision mesh" << std::endl;
//     // Pushback the collision object
//     vector_collision_objects.push_back(collision_object);
//   }

//   // SAMPLE AND POPULATE PLACEMENTS
//   LoadPlacements(num_surfaces);

//   // Display placements
//   display_placements_.header.frame_id = common_reference_;
//   for (const auto &object : objects_) {
//     for (const auto &surface : object.second.surfaces_) {
//       for (const auto &placement : surface.placements_) {
//         geometry_msgs::Pose pose;
//         tf::poseEigenToMsg(object.second.pose_ * placement, pose);
//         display_placements_.poses.push_back(pose);
//       }
//     }
//   }

//   auto object = objects_.at("stick_red1");
//   for (const auto &grasp : object.grasps_) {
//     geometry_msgs::Pose pose;
//     tf::poseEigenToMsg(grasp, pose);
//     display_placements_.poses.push_back(pose);
//   }
//   visualization_msgs::Marker display_object;
//   display_object.header.frame_id = common_reference_;
//   display_object.ns = "grasp";
//   display_object.id = display_objects_.markers.size();
//   display_object.type = visualization_msgs::Marker::MESH_RESOURCE;
//   std::string mesh_name = object.mesh_.substr(0, object.mesh_.find_last_of('.'));
//   display_object.mesh_resource = "file://" +
//                                  filename.substr(0, filename.substr(0, filename.find_last_of('/')).find_last_of('/')) +
//                                  "/meshes/" + mesh_name + ".dae";
//   display_object.action = visualization_msgs::Marker::ADD;
//   tf::poseEigenToMsg(Eigen::Affine3d::Identity(), display_object.pose);
//   display_object.frame_locked = true;
//   display_object.scale.x = 1;
//   display_object.scale.y = 1;
//   display_object.scale.z = 1;
//   if (object.name_.find("red") != std::string::npos) {
//     display_object.color.r = 1;
//     display_object.color.g = 0;
//     display_object.color.b = 0;
//     display_object.mesh_use_embedded_materials = false;
//   } else if (object.name_.find("green") != std::string::npos) {
//     display_object.color.r = 0;
//     display_object.color.g = 1;
//     display_object.color.b = 0;
//     display_object.mesh_use_embedded_materials = false;
//   } else if (object.name_.find("blue") != std::string::npos) {
//     display_object.color.r = 0;
//     display_object.color.g = 0;
//     display_object.color.b = 1;
//     display_object.mesh_use_embedded_materials = false;
//   } else {
//     display_object.color.r = 0.5;
//     display_object.color.g = 0.5;
//     display_object.color.b = 0.5;
//     display_object.mesh_use_embedded_materials = false;
//   }
//   display_object.color.a = 1;
//   display_objects_.markers.push_back(display_object);

//   for (const auto sop : object.stable_object_poses_) {
//     display_object.ns = "sop";
//     display_object.id = display_objects_.markers.size();
//     tf::poseEigenToMsg(sop, display_object.pose);
//     display_objects_.markers.push_back(display_object);
//   }

//   // Apply (syncronously) the created collision objects
//   planning_scene_interface_.applyCollisionObjects(vector_collision_objects);

//   LoadLocations();
//   std::cout << "Loaded Locations" << std::endl;

//   // compute the maximum allowed movement distance of the robot as the maximum of the shortest distances between
//   // each surface and the robot origin

//   PopulateLocationConnections();

//   // Display location coonections
//   visualization_msgs::Marker display_location_connections;
//   display_location_connections.header.frame_id = common_reference_;
//   display_location_connections.ns = "location_connections";
//   display_location_connections.id = display_objects_.markers.size();
//   display_location_connections.type = visualization_msgs::Marker::LINE_LIST;
//   display_location_connections.action = visualization_msgs::Marker::ADD;
//   tf::poseEigenToMsg(Eigen::Affine3d::Identity(), display_location_connections.pose);
//   display_location_connections.frame_locked = true;
//   display_location_connections.scale.x = 0.01;
//   display_location_connections.scale.y = 1;
//   display_location_connections.scale.z = 1;
//   display_location_connections.color.r = 1;
//   display_location_connections.color.g = 1;
//   display_location_connections.color.b = 0;
//   display_location_connections.color.a = 1;
//   for (std::size_t i = 0; i < base_locations_.size(); ++i) {
//     std::size_t hash = 0;
//     hasher_.CombineHashPose(hash, base_locations_.at(i));
//     auto connections = location_connections_.find(hash);
//     if (connections != location_connections_.end()) {
//       geometry_msgs::Point start;
//       tf::pointEigenToMsg(base_locations_.at(i).translation(), start);
//       for (auto j : connections->second) {
//         if (j > i) {
//           geometry_msgs::Point goal;
//           tf::pointEigenToMsg(base_locations_.at(j).translation(), goal);
//           display_location_connections.points.push_back(start);
//           display_location_connections.points.push_back(goal);
//           // std::cout << "Distance " << i << "-" << j << " " << hypot(goal.x-start.x,goal.y-start.y) << std::endl;
//         }
//       }
//     }
//   }
//   if (!display_location_connections.points.empty())
//     display_objects_.markers.push_back(display_location_connections);
//   display_objects_start_ = display_objects_;
//   // POPULATE ROBOT STATE
//   display_robot_state_.state.joint_state = robot_home_joint_config_;

//   std::cout << "World loaded" << std::endl;
// }

void NonMonotonicTampProblem::ComputeStateSketchFeatures(State *const state) {
  // compute state features
  auto casted_state = dynamic_cast<MoveitTampState *>(state);

  bool found_misplaced_green = false;
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    if (!objects_.at(object_names_.at(i)).moveable_ || object_names_.at(i).find("green") == std::string::npos) {
      continue;
    }
    if (Misplaced(object_names_.at(i), casted_state->GetObjectPoses().at(i))) {
      found_misplaced_green = true;
      break;
    } else {
      objects_.at(object_names_.at(i)).moveable_ = false;
    }
  }
  only_green_goals_ = found_misplaced_green;
  std::cout << "Only Green Goals: " << only_green_goals_ << " with grasped " << casted_state->GetAttatchedObject()
            << std::endl;

  SetMisplacedObjects(casted_state);
  SetBlockingObjects(casted_state, true);
}

void NonMonotonicTampProblem::SetMisplacedObjects(MoveitTampState *const state) const {
  std::unordered_set<std::size_t> state_misplaced_objects;
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    if (only_green_goals_ && object_names_.at(i).find("green") == std::string::npos) {
      continue;
    }
    if (state->GetAttatchedObject() != object_names_.at(i) &&
        Misplaced(object_names_.at(i), state->GetObjectPoses().at(i))) {
      state_misplaced_objects.insert(i);
    }
    if (state->GetAttatchedObject() == object_names_.at(i)) {
      const auto &misplaced_object = objects_.at(state->GetAttatchedObject());
      auto target_pose =
          goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
      if (target_pose != goal_positions_.end()) {
        const std::size_t num_blocking_objects =
            BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), *state->GetGrasp(),
                                 misplaced_object.stable_object_poses_, misplaced_object.name_, 1);
        if (num_blocking_objects > 0) {
          state_misplaced_objects.insert(i);
        }
      }
    }
  }
  // std::cout << "Misplaced are";
  // for (const auto &obj : state_misplaced_objects) {
  //   std::cout << " " << object_names_.at(obj);
  // } std::cout << std::endl;
  state->SetMisplacedObjects(state_misplaced_objects);
}

bool NonMonotonicTampProblem::Misplaced(const std::string &name, const Eigen::Affine3d &pose) const {

  auto pos = name.find("_");
  if (pos == std::string::npos) {
    return false;
  }
  auto iter = goal_positions_.find("target" + name.substr(pos));
  if (iter != goal_positions_.end()) {
    if (OnCircle(iter->second, goal_tolerance_radius_, pose.translation())) {
      return false;
    } else {
      return true;
    }

  } else {
    return false;
  }

  // if (name.find("stick_blue") != std::string::npos) {
  //   const auto &table3 = objects_.at("table3");
  //   if (table3.surfaces_.front().on(table3.pose_.inverse() * pose.translation())) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // } else if (name.find("stick_green") != std::string::npos) {
  //   const auto &table4 = objects_.at("table4");
  //   if (table4.surfaces_.front().on(table4.pose_.inverse() * pose.translation())) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // } else {
  //   return false;
  // }
}

// std::size_t NonMonotonicTampProblem::BlockingObjects(MoveitTampState const *const state,
//                                                      const Eigen::Affine3d &robot_pose,
//                                                      const Eigen::Affine3d &gripper_pose,
//                                                      const std::string &misplaced_object_name,
//                                                      const std::size_t max_num_blocking_objects) const {
//   Eigen::Vector3d o = gripper_pose.translation();
//   Eigen::Vector3d v = robot_pose.translation() - o;

//   std::size_t num_blocking_objects = 0;
//   for (const auto &obstructing_object : objects_) {
//     if (num_blocking_objects >= max_num_blocking_objects)
//       break;

//     if (!obstructing_object.second.moveable_ || state->GetAttatchedObject() == obstructing_object.second.name_ ||
//         obstructing_object.second.name_ == misplaced_object_name)
//       continue;

//     Eigen::Vector3d w =
//         state->GetObjectPoses().at(object_indices.at(obstructing_object.second.name_)).translation() - o;
//     // TODO: v(2) = w(2) = 0
//     double dist = (w - std::min(std::max(0.0, w.dot(v) / v.squaredNorm()), 1.0) * v).norm();

//     if (dist < blocking_object_distance_threshold_)
//       num_blocking_objects++;
//   }

//   return num_blocking_objects;
// }

// std::size_t NonMonotonicTampProblem::BlockingObjectsPick(MoveitTampState const *const state,
//                                                          const Eigen::Affine3d &object_pose,
//                                                          const Eigen::Affine3d &gripper_pose,
//                                                          const std::string &misplaced_object_name,
//                                                          const std::size_t max_num_blocking_objects) const {
//   std::size_t min_num_blocking_objects = max_num_blocking_objects;
//   for (const auto &robot_pose : base_locations_) {
//     if (min_num_blocking_objects == 0)
//       break;

//     if (!OnWorkspace(robot_pose, object_pose))
//       continue;

//     std::size_t num_blocking_objects =
//         BlockingObjects(state, robot_pose, gripper_pose, misplaced_object_name, min_num_blocking_objects);

//     if (num_blocking_objects < min_num_blocking_objects)
//       min_num_blocking_objects = num_blocking_objects;
//   }

//   return min_num_blocking_objects;
// }

// std::size_t NonMonotonicTampProblem::BlockingObjectsPlace(MoveitTampState const *const state,
//                                                           const Eigen::Affine3d &placement_pose,
//                                                           const Eigen::Affine3d &grasp,
//                                                           const std::vector<Eigen::Affine3d> &sops,
//                                                           const std::string &misplaced_object_name,
//                                                           const std::size_t max_num_blocking_objects) const {
//   std::size_t min_num_blocking_objects = max_num_blocking_objects;
//   for (const auto &robot_pose : base_locations_) {
//     if (min_num_blocking_objects == 0)
//       break;

//     if (!OnWorkspace(robot_pose, placement_pose))
//       continue;

//     for (const auto &sop : sops) {
//       if (min_num_blocking_objects == 0)
//         break;

//       std::size_t num_blocking_objects = BlockingObjects(state, robot_pose, placement_pose * sop * grasp,
//                                                          misplaced_object_name, min_num_blocking_objects);
//       if (num_blocking_objects < min_num_blocking_objects)
//         min_num_blocking_objects = num_blocking_objects;
//     }
//   }

//   return min_num_blocking_objects;
// }

// void NonMonotonicTampProblem::SetBlockingObjects(MoveitTampState *const state, bool compute_s) const {

//   std::size_t min_num_blocking_objects = std::numeric_limits<std::size_t>::max();
//   std::size_t sum_min_blocking_objects = 0;

//   for (const auto misplaced_object_index : state->GetMisplacedObjects()) {
//     if (!compute_s && min_num_blocking_objects == 0)
//       break;

//     const auto &misplaced_object = objects_.at(object_names_.at(misplaced_object_index));
//     auto target_pose = goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
//     if (target_pose == goal_positions_.end())
//       continue;

//     if (state->GetAttatchedObject() == misplaced_object.name_) {
//       std::size_t num_blocking_objects =
//           BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), *state->GetGrasp(),
//                                misplaced_object.stable_object_poses_, misplaced_object.name_,
//                                compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects);
//       if (num_blocking_objects < min_num_blocking_objects)
//         min_num_blocking_objects = num_blocking_objects;
//       if (compute_s)
//         sum_min_blocking_objects += num_blocking_objects;
//     } else {
//       const auto object_pose = state->GetObjectPoses().at(misplaced_object_index);
//       // std::cout << "Checking misplaced obj: " << misplaced_object.name_ << std::endl;

//       std::size_t min_num_object_blocking_objects =
//           compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects;
//       for (const auto &grasp : misplaced_object.grasps_) {
//         if (min_num_object_blocking_objects == 0)
//           break;

//         std::size_t num_blocking_objects = BlockingObjectsPick(state, object_pose, object_pose * grasp,
//                                                                misplaced_object.name_, min_num_object_blocking_objects);
//         if (min_num_object_blocking_objects > num_blocking_objects) {
//           num_blocking_objects +=
//               BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), grasp,
//                                    misplaced_object.stable_object_poses_, misplaced_object.name_,
//                                    min_num_object_blocking_objects - num_blocking_objects);
//         }

//         if (num_blocking_objects < min_num_object_blocking_objects) {
//           min_num_object_blocking_objects = num_blocking_objects;
//         }
//       }
//       if (min_num_object_blocking_objects < min_num_blocking_objects)
//         min_num_blocking_objects = min_num_object_blocking_objects;
//       if (compute_s)
//         sum_min_blocking_objects += min_num_object_blocking_objects;
//     }
//   }

//   if (min_num_blocking_objects > objects_.size())
//     min_num_blocking_objects = 0;

//   state->SetMinObstructingObjects(min_num_blocking_objects);
//   if (compute_s)
//     state->SetSumMinObjectsObstructingMisplacedObjects(sum_min_blocking_objects);
// }
