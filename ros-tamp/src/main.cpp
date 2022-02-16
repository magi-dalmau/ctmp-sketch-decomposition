#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <random>
#include <tinyxml.h>
#include <vector>

class BaseStateSpace {
  bool isValid(double x, double y, double yaw) const {
    return isfinite(x) && (x >= x_min) && (x <= x_max) && isfinite(y) && (y >= y_min) && (y <= y_max) && isfinite(yaw);
  }
  double x_min, y_min, x_max, y_max;
}

class Object {
public:
  Object(const Eigen::Affine3d &object_pose) : pose(object_pose){};
  Eigen::Affine3d pose;
};

class SupportingSurface {
public:
  SupportingSurface(double x_min, double x_max, double y_min, double y_max, const Eigen::Affine3d &surface_pose)
      : pose(surface_pose) {
    min(0) = x_min;
    min(1) = y_min;
    max(0) = x_max;
    max(1) = y_max;
  };
  Eigen::Affine3d pose;
  Eigen::Vector2d min, max;
  std::vector<Eigen::Affine3d> placements;
  double area() const { return (max - min).prod(); }
  bool on(const Eigen::Vector3d &position) const {
    const auto v = pose.inverse() * position;
    const double tol = 1e-6;
    return (v(0) >= min(0) - tol && v(0) <= max(0) + tol) && (v(1) >= min(1) - tol && v(1) <= max(1) + tol) &&
           (v(2) >= 0. - tol && v(2) <= 0. + tol);
  }
  void sample() {
    const unsigned int num_bins = std::max(1., std::ceil(sqrt(double(placements.size()))));
    std::vector<double> weights_x(num_bins, 0), weights_y(num_bins, 0), weights_yaw(num_bins, 0);
    std::vector<double> intervals_x(num_bins + 1), intervals_y(num_bins + 1), intervals_yaw(num_bins + 1);
    double x, y, yaw;
    for (unsigned int i = 0; i <= num_bins; ++i) {
      intervals_x.at(i) = min(0) + (max(0) - min(0)) * double(i) / double(num_bins);
      intervals_y.at(i) = min(1) + (max(1) - min(1)) * double(i) / double(num_bins);
      intervals_yaw.at(i) = -M_PI + 2. * M_PI * double(i) / double(num_bins);
    }
    for (const auto &placement : placements) {
      const auto local_pose = pose.inverse() * placement;
      weights_x.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(0) - min(0)) /
                                                                   (max(0) - min(0)) * double(num_bins))))++;
      weights_y.at(std::min(num_bins - 1, (unsigned int)std::floor((local_pose.translation()(1) - min(1)) /
                                                                   (max(1) - min(1)) * double(num_bins))))++;
      weights_yaw.at(std::min(num_bins - 1,
                              (unsigned int)std::floor((atan2(local_pose.matrix()(1, 0) - local_pose.matrix()(0, 1),
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
    placements.push_back(pose * Eigen::Translation3d(distribution_x(generator), distribution_y(generator), 0) *
                         Eigen::AngleAxisd(distribution_yaw(generator), Eigen::Vector3d::UnitZ()));
  }
};

int main(int argc, char **argv) {
  const std::string filename =
      "/ros_ws/src/ros-tamp/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/problem_definitions/pb_3.xml";
  TiXmlDocument doc(filename);
  doc.LoadFile();
  if (!doc.LoadFile()) {
    std::cout << "Failed to load file \"" << filename << "\"." << std::endl;
    return 0;
  }

  std::map<std::string, Object> objects;
  std::map<std::string, std::vector<SupportingSurface>> surfaces;

  TiXmlHandle h_doc(&doc);
  for (auto obj = h_doc.FirstChildElement("problem").FirstChildElement("objects").FirstChildElement("obj").ToElement();
       obj; obj = obj->NextSiblingElement("obj")) {
    const std::string name = obj->FirstChildElement("name")->GetText();
    const std::string moveable = obj->FirstChildElement("moveable")->GetText();
    std::vector<std::string> values;
    boost::split(values, std::string(obj->FirstChildElement("pose")->GetText()), boost::is_any_of(" "));
    Eigen::Affine3d pose;
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 4; ++j)
        pose.matrix()(i, j) = std::stod(values.at(i * 4 + j));

    if (boost::algorithm::to_lower_copy(moveable).compare("true") == 0 || moveable.compare("1") == 0) {
      objects.insert(std::make_pair(name, Object(pose)));
    } else {
      for (auto sssp = obj->FirstChildElement("sssp"); sssp; sssp = sssp->NextSiblingElement("sssp")) {
        const double x_min = std::stod(sssp->FirstChildElement("xmin")->GetText());
        const double x_max = std::stod(sssp->FirstChildElement("xmax")->GetText());
        const double y_min = std::stod(sssp->FirstChildElement("ymin")->GetText());
        const double y_max = std::stod(sssp->FirstChildElement("ymax")->GetText());
        const double z = std::stod(sssp->FirstChildElement("zmin")->GetText());
        surfaces[name].push_back(SupportingSurface(x_min, x_max, y_min, y_max, pose * Eigen::Translation3d(0, 0, z)));
      }
      if (obj->FirstChildElement("attachments")) {
        for (auto attachment = obj->FirstChildElement("attachments")->FirstChildElement("name"); attachment;
             attachment = attachment->NextSiblingElement("name")) {
          const auto &object_pose = objects.at(attachment->GetText()).pose;
          for (auto &surface : surfaces.at(name)) {
            if (surface.on(object_pose.translation()))
              surface.placements.push_back(object_pose);
          }
        }
      }
    }
  }

  //   for (const auto &object : objects) {
  //     std::cout << object.first << std::endl;
  //     std::cout << object.second.pose.matrix() << std::endl;
  //     std::cout << std::endl;
  //   }

  //   for (const auto &surface_set : surfaces) {
  //     for (const auto &surface : surface_set.second) {
  //       std::cout << surface_set.first << " " << surface.min.transpose() << " " << surface.max.transpose() << " "
  //                 << surface.placements.size() << std::endl;
  //       std::cout << surface.pose.matrix() << std::endl;
  //       std::cout << std::endl;
  //     }
  //   }

  unsigned int num_surfaces = 0;
  for (const auto &surface_set : surfaces)
    num_surfaces += surface_set.second.size();

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  std::vector<double> weights(num_surfaces);
  for (unsigned int k = 0; k < 50; ++k) {
    unsigned int i = 0;
    for (const auto &surface_set : surfaces)
      for (const auto &surface : surface_set.second) {
        weights.at(i++) = surface.area() / (1e-6 + double(surface.placements.size()));
        // std::cout << surface.placements.size() << " ";
      }
    // std::cout << std::endl;
    std::discrete_distribution<std::size_t> distribution(weights.begin(), weights.end());
    unsigned int j = distribution(generator);
    i = 0;
    for (auto &surface_set : surfaces) {
      if (i + surface_set.second.size() <= j) {
        i += surface_set.second.size();
      } else {
        surface_set.second.at(j - i).sample();
        break;
      }
    }
  }

  BaseStateSpace base_space;
  base_space.x_min = -2.5;
  base_space.y_min = -2.5;
  base_space.x_max = 2.5;
  base_space.y_max = 2.5;
  double dist = 0.5;

  for (const auto &surface_set : surfaces) {
    for (const auto &surface : surface_set.second) {
      std::vector<double> weights(4);
      weights(0) = weights(2) = surface.max(0) - surface.min(0);
      weights(1) = weights(3) = surface.max(1) - surface.min(1);
      std::discrete_distribution<std::size_t> disc_distr(weights.begin(), weights.end());
      std::uniform_real_distribution<double> unif_distr;
      std::normal_distribution<double> normal_distr;
      unsigned int i = disc_distr(generator);
      double u = unif_distr(generator);
      double z = normal_distr(generator);
      Eigen::Vector3d pos;

    }
  }

  return 0;
}