#pragma once
#include <Eigen/Geometry>
#include <action.hpp>

class MoveBaseAction : public Action {
public:
  MoveBaseAction(const Eigen::Affine3d &target_location) : Action() {
    action_id_ = "MOVE_BASE";
    target_location_ = target_location;
  };

  virtual Action *Clone() const { return new MoveBaseAction(target_location_); };

  void print(std::ostream &os) const override {

    os << action_id_ << " to position: " << target_location_.translation().transpose()
       << " and orientation: " << Eigen::Quaterniond(target_location_.rotation()).coeffs().transpose();
  }

  std::string GetActionId() const { return action_id_; }
  Eigen::Affine3d GetTargetLocation() const { return target_location_; }
  void AddViaPoint(const Eigen::Affine3d &point) { via_points_.push_back(point); }
  std::vector<Eigen::Affine3d> GetViaPoints() const { return via_points_; }

protected:
  std::string action_id_;
  Eigen::Affine3d target_location_;
  std::vector<Eigen::Affine3d> via_points_;
};