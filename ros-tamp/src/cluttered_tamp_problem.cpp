#include <cluttered_tamp_problem.hpp>

ClutteredTampProblem::ClutteredTampProblem(const std::string &filename, const std::string &planning_group,
                                           ros::NodeHandle *nodehandle)
    : MoveitTampProblem(filename, planning_group, nodehandle) {
  // Load world
  LoadWorld(filename);
  goal_region_=nh_.param("goal_region",true);
  if(goal_region_){
    std::string region, object;
    std::size_t num_regions = 3;
    for (size_t i = 1; i <= num_regions; i++) {
      object = nh_.param("object_region_" + std::to_string(i), std::string(""));
      region = nh_.param("region_" + std::to_string(i), std::string(""));
      if (!region.empty() && !object.empty()) {
        region_goals_.insert(std::make_pair(object, region));
      }
    }
    goal_positions_.clear();
  }
  std::cout<<"Single-position goals amount is: "<<goal_positions_.size()<<std::endl;
  
  

  ROS_DEBUG("PROBLEM INITIALIZED");
}

// void ClutteredTampProblem::AdaptativeSampling(const State *const state) {
//   for (const auto &name : supporting_object_names) {
//     auto supporting_obj = objects_.find(name);
//     if (supporting_obj != objects_.end()) {
//       for (auto &surface : supporting_obj->second.surfaces_) {
//         surface.placements_.clear();
//       }
//     }
//   }

//   auto casted_state = dynamic_cast<const MoveitTampState *>(state);
//   AddCurrentObjectsToPlacements(casted_state->GetObjectPoses());

//   for (const auto &name : supporting_object_names) {
//     auto supporting_obj = objects_.find(name);
//     if (supporting_obj != objects_.end()) {
//       for (auto &surface : supporting_obj->second.surfaces_) {
//         for (size_t i = 0; i < placements_per_table_adaptative_sampling_; i++) {
//           surface.sample();
//         }
//       }
//     }
//   }
// }

bool ClutteredTampProblem::Misplaced(const std::string &name, const Eigen::Affine3d &pose) const {
  
  if (goal_region_){
    static char const *digits = "0123456789";
    std::size_t const n = name.find_first_of(digits);
    // TODO: It is assumed that all the object that could be a goal have a number in the name
    // std::cout << "Checking if " << name.substr(0, n) << " is misplaced" << std::endl;
    if (n != std::string::npos) {
      auto goal = region_goals_.find(name.substr(0, n));
      if (goal != region_goals_.end()) {
        const auto &table = objects_.at(goal->second);
        if (table.surfaces_.front().on(table.pose_.inverse() * pose.translation())) {
          return false;
        } else {
          return true;
        }
      } else {
        return false;
      }
    }
    return false;
  }else{
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

// void ClutteredTampProblem::SetBlockingObjects(MoveitTampState *const state, bool compute_s) const {

//   std::size_t min_num_blocking_objects = std::numeric_limits<std::size_t>::max();
//   std::size_t sum_min_blocking_objects = 0;

//   for (const auto misplaced_object_index : state->GetMisplacedObjects()) {
//     if (!compute_s && min_num_blocking_objects == 0)
//       break;

//     const auto &misplaced_object = objects_.at(object_names_.at(misplaced_object_index));
//     const auto object_pose = state->GetObjectPoses().at(misplaced_object_index);

//     std::size_t min_num_object_blocking_objects =
//         compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects;
//     for (const auto &grasp : misplaced_object.grasps_) {
//       if (min_num_object_blocking_objects == 0)
//         break;

//       Eigen::Vector3d o = object_pose * grasp.translation();

//       for (const auto &robot_pose : base_locations_) {
//         if (min_num_object_blocking_objects == 0)
//           break;

//         if (!OnWorkspace(robot_pose, object_pose))
//           continue;

//         Eigen::Vector3d v = robot_pose.translation() - o;

//         std::size_t num_blocking_objects = 0;
//         for (const auto &obstructing_object : objects_) {
//           if (num_blocking_objects >= min_num_object_blocking_objects)
//             break;

//           if (!obstructing_object.second.moveable_ || state->GetAttatchedObject() == obstructing_object.second.name_ ||
//               obstructing_object.second.name_ == misplaced_object.name_)
//             continue;

//           Eigen::Vector3d w =
//               state->GetObjectPoses().at(object_indices.at(obstructing_object.second.name_)).translation() - o;

//           // TODO: v(2) = w(2) = 0
//           double dist = (w - std::min(std::max(0.0, w.dot(v) / v.squaredNorm()), 1.0) * v).norm();

//           if (dist < blocking_object_distance_threshold_)
//             num_blocking_objects++;
//         }
//         if (num_blocking_objects < min_num_object_blocking_objects)
//           min_num_object_blocking_objects = num_blocking_objects;
//       }
//     }
//     if (min_num_object_blocking_objects < min_num_blocking_objects)
//       min_num_blocking_objects = min_num_object_blocking_objects;
//     if (compute_s)
//       sum_min_blocking_objects += min_num_object_blocking_objects;
//   }
//   if (min_num_blocking_objects > objects_.size())
//     min_num_blocking_objects = 0;

//   state->SetMinObstructingObjects(min_num_blocking_objects);
//   if (compute_s)
//     state->SetSumMinObjectsObstructingMisplacedObjects(sum_min_blocking_objects);
// }

void ClutteredTampProblem::SetBlockingObjects(MoveitTampState *const state, bool compute_s) const {

  std::size_t min_num_blocking_objects = std::numeric_limits<std::size_t>::max();
  std::size_t sum_min_blocking_objects = 0;

  for (const auto misplaced_object_index : state->GetMisplacedObjects()) {
    if (!compute_s && min_num_blocking_objects == 0)
      break;

    const auto &misplaced_object = objects_.at(object_names_.at(misplaced_object_index));
    auto target_pose = goal_positions_.find("target" + misplaced_object.name_.substr(misplaced_object.name_.find("_")));
    if (target_pose == goal_positions_.end())
      continue;

    if (state->GetAttatchedObject() == misplaced_object.name_) {
      std::size_t num_blocking_objects =
          BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), *state->GetGrasp(),
                               misplaced_object.stable_object_poses_, misplaced_object.name_,
                               compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects);
      if (num_blocking_objects < min_num_blocking_objects)
        min_num_blocking_objects = num_blocking_objects;
      if (compute_s)
        sum_min_blocking_objects += num_blocking_objects;
    } else {
      const auto object_pose = state->GetObjectPoses().at(misplaced_object_index);
      // std::cout << "Checking misplaced obj: " << misplaced_object.name_ << std::endl;

      std::size_t min_num_object_blocking_objects =
          compute_s ? std::numeric_limits<std::size_t>::max() : min_num_blocking_objects;
      for (const auto &grasp : misplaced_object.grasps_) {
        if (min_num_object_blocking_objects == 0)
          break;

        std::size_t num_blocking_objects = BlockingObjectsPick(state, object_pose, object_pose * grasp,
                                                               misplaced_object.name_, min_num_object_blocking_objects);
        if (min_num_object_blocking_objects > num_blocking_objects) {
          num_blocking_objects +=
              BlockingObjectsPlace(state, Eigen::Affine3d(Eigen::Translation3d(target_pose->second)), grasp,
                                   misplaced_object.stable_object_poses_, misplaced_object.name_,
                                   min_num_object_blocking_objects - num_blocking_objects);
        }

        if (num_blocking_objects < min_num_object_blocking_objects) {
          min_num_object_blocking_objects = num_blocking_objects;
        }
      }
      if (min_num_object_blocking_objects < min_num_blocking_objects)
        min_num_blocking_objects = min_num_object_blocking_objects;
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

std::size_t ClutteredTampProblem::BlockingObjectsPlace(MoveitTampState const *const state,
                                                       const Eigen::Affine3d &placement_pose,
                                                       const Eigen::Affine3d &grasp,
                                                       const std::vector<Eigen::Affine3d> &sops,
                                                       const std::string &misplaced_object_name,
                                                       const std::size_t max_num_blocking_objects) const {
  std::size_t min_num_blocking_objects = max_num_blocking_objects;
  for (const auto &robot_pose : base_locations_) {
    if (min_num_blocking_objects == 0)
      break;

    if (!OnWorkspace(robot_pose, placement_pose))
      continue;

    for (const auto &sop : sops) {
      if (min_num_blocking_objects == 0)
        break;

      std::size_t num_blocking_objects = BlockingObjects(state, robot_pose, placement_pose * sop * grasp,
                                                         misplaced_object_name, min_num_blocking_objects);
      if (num_blocking_objects < min_num_blocking_objects)
        min_num_blocking_objects = num_blocking_objects;
    }
  }

  return min_num_blocking_objects;
}

std::size_t ClutteredTampProblem::BlockingObjectsPick(MoveitTampState const *const state,
                                                      const Eigen::Affine3d &object_pose,
                                                      const Eigen::Affine3d &gripper_pose,
                                                      const std::string &misplaced_object_name,
                                                      const std::size_t max_num_blocking_objects) const {
  std::size_t min_num_blocking_objects = max_num_blocking_objects;
  for (const auto &robot_pose : base_locations_) {
    if (min_num_blocking_objects == 0)
      break;

    if (!OnWorkspace(robot_pose, object_pose))
      continue;

    std::size_t num_blocking_objects =
        BlockingObjects(state, robot_pose, gripper_pose, misplaced_object_name, min_num_blocking_objects);

    if (num_blocking_objects < min_num_blocking_objects)
      min_num_blocking_objects = num_blocking_objects;
  }

  return min_num_blocking_objects;
}

std::size_t ClutteredTampProblem::BlockingObjects(MoveitTampState const *const state, const Eigen::Affine3d &robot_pose,
                                                  const Eigen::Affine3d &gripper_pose,
                                                  const std::string &misplaced_object_name,
                                                  const std::size_t max_num_blocking_objects) const {
  Eigen::Vector3d o = gripper_pose.translation();
  Eigen::Vector3d v = robot_pose.translation() - o;

  std::size_t num_blocking_objects = 0;
  for (const auto &obstructing_object : objects_) {
    if (num_blocking_objects >= max_num_blocking_objects)
      break;

    if (!obstructing_object.second.moveable_ || state->GetAttatchedObject() == obstructing_object.second.name_ ||
        obstructing_object.second.name_ == misplaced_object_name)
      continue;

    Eigen::Vector3d w =
        state->GetObjectPoses().at(object_indices.at(obstructing_object.second.name_)).translation() - o;
    // TODO: v(2) = w(2) = 0
    double dist = (w - std::min(std::max(0.0, w.dot(v) / v.squaredNorm()), 1.0) * v).norm();

    if (dist < blocking_object_distance_threshold_)
      num_blocking_objects++;
  }

  return num_blocking_objects;
}
