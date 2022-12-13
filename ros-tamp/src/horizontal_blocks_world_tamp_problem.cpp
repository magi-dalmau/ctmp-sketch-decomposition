#include <horizontal_blocks_world_tamp_problem.hpp>

BlocksWorldTampProblem::BlocksWorldTampProblem(const std::string &filename, const std::string &planning_group,
                                               ros::NodeHandle *nodehandle)
    : MoveitTampProblem(filename, planning_group, nodehandle) {
  goal_tolerance_radius_ = nh_.param("goal_tolerance_radius_", 0.005);
  // Load world

  LoadWorld(filename);
  // for (const auto goal : goal_positions_){
  //   std::cout<<"Goal : "<<goal.first<<" location : "<<goal.second.transpose()<<std::endl;
  // }
  throw std::runtime_error("Stoped here for debugging");
  std::cout << "NUM OF TARGET GOALS IS: " << goal_positions_.size() << std::endl;
  ROS_DEBUG("PROBLEM INITIALIZED");
}

void BlocksWorldTampProblem::ComputeStateSketchFeatures(State *const state) {
  // compute state features
  auto casted_state = dynamic_cast<MoveitTampState *>(state);

  bool found_misplaced_green = false;
  // for (std::size_t i = 0; i < objects_.size(); ++i) {
  //   if (!objects_.at(object_names_.at(i)).moveable_ || object_names_.at(i).find("green") == std::string::npos) {
  //     continue;
  //   }
  //   if (Misplaced(object_names_.at(i), casted_state->GetObjectPoses().at(i))) {
  //     found_misplaced_green = true;
  //     break;
  //   } else {
  //     objects_.at(object_names_.at(i)).moveable_ = false;
  //   }
  // }
  only_green_goals_ = found_misplaced_green;
  // std::cout << "Only Green Goals: " << only_green_goals_ << " with grasped " << casted_state->GetAttatchedObject()
  // << std::endl;

  SetMisplacedObjects(casted_state);
  SetBlockingObjects(casted_state, true);
}

void BlocksWorldTampProblem::SetMisplacedObjects(MoveitTampState *const state) const {
  // std::cout << " Computing misplaced..." << std::endl;
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
  // std::cout << " End misplaced compute" << std::endl;
}

bool BlocksWorldTampProblem::Misplaced(const std::string &name, const Eigen::Affine3d &pose) const {

  auto pos = name.find("_");
  if (pos == std::string::npos) {
    return false;
  }
  auto iter = goal_positions_.find("target" + name.substr(pos));
  if (iter != goal_positions_.end()) {
    Eigen::Vector3d alternative(iter->second.x() + 0.04, iter->second.y(),
                                iter->second.z() + 0.242); // TODO: Provisional per red objects girats al principi

    if (OnCircle(iter->second, goal_tolerance_radius_, pose.translation()) ||
        OnCircle(alternative, goal_tolerance_radius_, pose.translation())) {
      return false;
    } else {
      // std::cout << "Object " << name << " is misplaced pose: " << pose.translation().transpose()
      //           << " target: " << iter->second.transpose() << std::endl;
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
