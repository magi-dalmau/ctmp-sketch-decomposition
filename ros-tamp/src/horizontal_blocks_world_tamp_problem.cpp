#include <algorithm>
#include <horizontal_blocks_world_tamp_problem.hpp>
#include <queue>

BlocksWorldTampProblem::BlocksWorldTampProblem(const std::string &filename, const std::string &planning_group,
                                               ros::NodeHandle *nodehandle)
    : MoveitTampProblem(filename, planning_group, nodehandle) {
  goal_tolerance_radius_ = nh_.param("goal_tolerance_radius_", 0.005);
  // Load world

  LoadWorld(filename);
  LoadAndSetGoals();
  for (const auto goal : goal_positions_) {
    std::cout << "Goal : " << goal.first << " location : " << goal.second.transpose() << std::endl;
  }
  std::cout << "NUM OF TARGET GOALS IS: " << goal_positions_.size() << std::endl;
  // throw std::runtime_error("Stoped here for debugging");

  // initialize object neighbours template. WARNING it is assumed that the set of objects does not change during the
  // whole problem
  std::size_t num_of_objects = objects_.size();
  for (std::size_t i = 0; num_of_objects; i++) {
    object_neighbours_template_.insert(std::make_pair(i, std::make_pair(num_of_objects, num_of_objects)));
  }
  //TODO define neighbourh translation
  left2right_pose_ = Eigen::Affine3d::fromPositionOrientationScale(Eigen::Vector3d(0.05, 0, 0),
                                                                   Eigen::Quaterniond(1., 0., 0., 0.), 1.);

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
  /*
  New PIPELINE:
  1)Get goal sequence (should stored in a class var)
  2)Identify current longest valid sequence
  2.1)"Cluster" objects that are near and well-oriented (i.e. that could be considered a sequence)
  2.2)Traverse all the clusters, starting from the biggers, identify the sequences present in the cluster (note that you
  could gave a cluster of near objects that, for instance, are placed in a cross shape and then you could have 2
  sequences ) 2.3)In each cluster, traverse all the sequences and check sequence validity. Keep the longest valid
  sequence in memory. Skip those new sequences that are worst than the keeped in memory. Skeep those clusters that have
  less objects than the longest sequence that is kept in memory. 3) Then an object is misplaced if: 3.1 AND (3.2 OR 3.3
  ) 3.1 Is a block that should be in the goal sequence (For instance, if we want to form a word with some letters, it
  could be the case that a letter it is not included in the word or it could be de case than a letter is already present
  in the word so the "duplicated"  letter is not needed) 3.2 Is standing, and it does'nt belong to the longest valid
  sequence 3.3 Is held, and its goal (place) is blocked
  **NOTES:
  Should exist an structure to save a sequence, an std::vector? The scope should be this function?

  */
 const auto object_poses=state->GetObjectPoses();
  std::size_t num_objects = objects_.size();
  // Search and set object neighbours (left and right)
  auto object_neighbours = object_neighbours_template_;
  for (std::size_t id_obj_left = 0; id_obj_left < num_objects; ++id_obj_left) {
    for (std::size_t id_obj_right = 0; id_obj_right < num_objects; ++id_obj_right) {
      if (IsNeighbour(object_poses.at(id_obj_left), object_poses.at(id_obj_right))) {
        object_neighbours.at(id_obj_left).second = id_obj_right;
        object_neighbours.at(id_obj_right).first = id_obj_left;
      }
    }
  }
  std::priority_queue<std::vector<std::size_t>, std::vector<std::vector<std::size_t>>, CompareSequences> sequences;

  // std::vector<std::vector<std::size_t>> sequences;
  for (std::size_t id_obj = 0; id_obj < num_objects; ++id_obj) {
    if (object_neighbours.at(id_obj).first >=
        num_objects) // Note that the template is initialized with id_obj:= num obj and thus if at this point this
                     // id has not been changed, this object does not have left (in this case) neighbour. If the
                     // object does not have left neighbour is "head of sequence"
    {
      std::vector<std::size_t> seq;
      seq.push_back(id_obj);
      std::size_t next = object_neighbours.at(id_obj).second;
      while (next < num_objects) {
        seq.push_back(next);
        next = object_neighbours.at(next).second;
      }
      sequences.push(seq);
    }
  }
  // Find the longest valid sequences
  std::vector<std::size_t> longest_valid_seq;
  bool valid = false;
  while (!valid && !sequences.empty()) {
    if (IsValidSequence(sequences.top())) {
      valid = true;
      longest_valid_seq = sequences.top();
    } else {
      sequences.pop();
    }
  }

  // std::cout << " Computing misplaced..." << std::endl;
  std::unordered_set<std::size_t> state_misplaced_objects;
  for (std::size_t i = 0; i < objects_.size(); ++i) {

    if (state->GetAttatchedObject() != object_names_.at(i) &&
        MisplacedBlocks(object_names_.at(i), state->GetObjectPoses().at(i), longest_valid_seq)) {
      state_misplaced_objects.insert(i);
    }
    if (state->GetAttatchedObject() == object_names_.at(i)) {
      const auto &misplaced_object = objects_.at(state->GetAttatchedObject());
      // TODO: Target pose, s'hauria de calcular relativa a la sequencia llarga
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
// TODO: Temp issue solving: Misplaced does not originally have as input the longest valid seq needed for this kind of
// problems
bool BlocksWorldTampProblem::Misplaced(const std::string &name, const Eigen::Affine3d &pose) const { return false; }

bool BlocksWorldTampProblem::MisplacedBlocks(const std::string &name, const Eigen::Affine3d &pose,
                                             const std::vector<std::size_t> &longest_valid_seq) const {
  /*
  Misplaced def:
  Oject is misplaced if: 3.1 AND (3.2 OR 3.3
    ) 3.1 Is a block that should be in the goal sequence (For instance, if we want to form a word with some letters, it
    could be the case that a letter it is not included in the word or it could be de case than a letter is already:
      CHECK 1: Object_name in Sequence specification
      CHECK 2:
  present in the word so the "duplicated"  letter is not needed) 3.2 Is standing, and it does'nt belong to the longest
  valid sequence 3.3 Is held, and its goal (place) is blocked
  */
  std::size_t object_idx = object_indices.at(name);
  std::string block_class = GetObjectClass(name); 
  auto pos_seq = goal_sequence_.find(block_class);
  if (pos_seq == std::string::npos) {
    return false; // Object not misplaced since it is no required in the goal seq
  }
  auto n = std::size_t(std::count(goal_sequence_.begin(), goal_sequence_.end(), block_class));

  std::size_t count = 0;
  for (const auto element : longest_valid_seq) {
    if (element == object_idx) {
      return false; // it is in the longest sequence so it is not misplaced
    }
    auto el_class = GetObjectClass(object_names_.at(element));
    if (block_class == el_class)
      count++;
  }
  if (count == n)
    return false; // although it is not in the longest sequence, this block is not needed because other blocks with the
                  // same id are already in the longest sequence and no more are needed.
  return true;

  // FINS ARA
  //  auto pos = name.find("_");
  //  if (pos == std::string::npos) {
  //      return false;
  //    }
  //    auto iter = goal_positions_.find("target" + name.substr(pos));
  //    if (iter != goal_positions_.end()) {
  //      Eigen::Vector3d alternative(iter->second.x() + 0.04, iter->second.y(),
  //                                  iter->second.z() + 0.242); // TODO: Provisional per red objects girats al principi

  //     if (OnCircle(iter->second, goal_tolerance_radius_, pose.translation()) ||
  //         OnCircle(alternative, goal_tolerance_radius_, pose.translation())) {
  //       return false;
  //     } else {
  //       // std::cout << "Object " << name << " is misplaced pose: " << pose.translation().transpose()
  //       //           << " target: " << iter->second.transpose() << std::endl;
  //       return true;
  //     }

  //   } else {
  //     return false;
  //   }
  // FINS AQUI
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

bool BlocksWorldTampProblem::LoadAndSetGoals() {
  goal_sequence_ = nh_.param("target_sequence", std::string("CTMP"));
  return true;

  // std::string object_names_root = "cube_";
  // std::string target_name_root = "target_";
  // Eigen::Affine3d origin_object_pose;
  // std::string target_sequence = nh_.param("target_sequence", std::string("CTMP"));
  // std::string origin_object_name = object_names_root + target_sequence.at(0);
  // double objects_width = 0.4; // TODO get dynamically
  // double inter_object_margin = 0.01;
  // for (std::size_t i = 0; i < target_sequence.size(); i++) {

  //   auto object = objects_.find(object_names_root + target_sequence.at(i));
  //   if (object != objects_.end()) {
  //     if (i == 0) {
  //         origin_object_pose = object->second.pose_;
  //         goal_positions_.insert(std::make_pair(target_name_root + target_sequence.at(i),
  //                                               origin_object_pose * Eigen::Vector3d(0, 0, 0.001)));
  //       } else {
  //         goal_positions_.insert(std::make_pair(
  //             target_name_root + target_sequence.at(i),
  //             origin_object_pose * Eigen::Vector3d(double(i) * (objects_width + inter_object_margin), 0, 0.001)));
  //       }
  //     } else {
  //       return false;
  //     }
  //   }
  //   return true;
}

bool BlocksWorldTampProblem::IsNeighbour(const Eigen::Affine3d &pose_obj_left,
                                        const Eigen::Affine3d &pose_obj_right) const {

  return (((pose_obj_left * left2right_pose_).matrix() - pose_obj_right.matrix()).norm() < 1e-3);
}

bool BlocksWorldTampProblem::IsValidSequence(const std::vector<std::size_t> &seq) const {
  // TODO: chop into valid suqsequence, now assuming one single seq to validate
  if (seq.size() > goal_sequence_.size())
    return false;

  std::string block_id = GetObjectId(seq.at(0));
  auto pos_seq = goal_sequence_.find(block_id);
  if (pos_seq == std::string::npos) {
    return false; // Object id is not in the goal seq
  }
  std::size_t i = 1; // TODO: put it more elegant
  while (pos_seq != std::string::npos && i < seq.size()) {
    for (i = 1; (i < seq.size() && (i + pos_seq) < goal_sequence_.size()); i++) {
      if (GetObjectId(seq.at(i)) != std::to_string(goal_sequence_.at(pos_seq + i))) {
        break;
      }
    }
    if (i < seq.size())
      pos_seq = (goal_sequence_.substr(pos_seq)).find(block_id);
  }
  return (pos_seq != std::string::npos &&
          i < seq.size()); // Si ha arribat al final vol dir que ha comprovat tota la sequencia

  // TODO: check the phisical space to place the full sequence
}

std::string BlocksWorldTampProblem::GetObjectId(const std::string &object_name) const {
//Example from: cube_T_2 -> return: T_2
  auto pos = object_name.find("_");
  if (pos == std::string::npos) {
    throw std::runtime_error("Cannot extract the object id from: " + object_name);
  }
  return object_name.substr(pos);
}

std::string BlocksWorldTampProblem::GetObjectId(const std::size_t &object_index) const {
  return GetObjectId(object_names_.at(object_index));
}

std::string BlocksWorldTampProblem::GetObjectClass(const std::size_t &object_index) const {
  return GetObjectClass(object_names_.at(object_index));
}
std::string BlocksWorldTampProblem::GetObjectClass(const std::string &object_name) const {
  // Example from: cube_T_2 -> return: T
  auto obj_id = GetObjectId(object_name); //e.g. This goes from cube_T_2 to T_2
  auto pos = obj_id.find("_");
  if (pos == std::string::npos) {
    throw std::runtime_error("Cannot extract the object id from: " + object_name);
  }
  return object_name.substr(0,pos);//E.g. this goes from T_2 to T
}

bool BlocksWorldTampProblem::AllGoalRegionBlockMisplaced(const std::string &object_name,
                                                         MoveitTampState const *const state) const {
                                                          //TODO REIMPLEMNT + IMPLEMENT GOAL TARGETS
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
