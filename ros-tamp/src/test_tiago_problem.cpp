#include <brfs.hpp>
#include <iostream>
#include <iwk.hpp>
#include <moveit_tamp_problem.hpp>
#include <random>
#include <ros/ros.h>
#include <siwr.hpp>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "test_tiago_problem_node");
  ROS_INFO("Starting Tiago problem test");
  // Declare Node Handle
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool lazy = nh.param("lazy", true);
  std::string planning_group = nh.param("planning_group", std::string("arm_torso"));
  std::cout << "The problem will be initialized" << std::endl;
  std::string problem_name = nh.param("problem_name", std::string("pb_3"));
  const std::string filename =
      "/ros_ws/src/ros-tamp/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/problem_definitions/" +
      problem_name + ".xml";

  auto problem = new MoveitTampProblem(filename, planning_group, &nh);

  std::cout << "My problem is:\n" << *problem << std::endl;

  // TEST BrFS
  //  auto brfs = new BrFS(problem);

  // Plan plan;
  // if (brfs->Solve(plan, lazy)) {
  //   std::cout << "Found solution" << std::endl;
  //   std::cout << plan << std::endl;
  //   while (ros::ok()) {
  //     problem->ExecutePlan(plan);
  //   }
  // } else {
  //   std::cout << "Solution not found" << std::endl;
  // }

  // TEST IWk

  // auto iwk = new IWk(problem, nh.param("k_max", 1));

  // Plan plan;
  // if (iwk->Solve(plan, lazy)) {
  //   std::cout << "Found solution" << std::endl;
  //   std::cout << plan << std::endl;
  //   while (ros::ok()) {
  //     problem->ExecutePlan(plan);
  //   }
  // } else {
  //   std::cout << "Solution not found" << std::endl;
  // }

  // TEST SIWr

  auto siwr = new SIWr(problem, nh.param("k_max", 1));
  ros::Time init_time = ros::Time::now();
  Plan plan;
  if (siwr->Solve(plan, lazy)) {
    auto solved_time = ros::Time::now();
    std::cout << "Found solution in " << ((solved_time - init_time).toSec()) / 60.0 << " min" << std::endl;
    std::cout << plan << std::endl;
    while (ros::ok()) {
      problem->ExecutePlan(plan);
    }
  } else {
    auto solved_time = ros::Time::now();
    std::cout << "Solution not found after " << ((solved_time - init_time).toSec()) / 60.0 << " min" << std::endl;
  }

  // END
  ros::waitForShutdown();
  delete problem;
  // delete brfs;
  // delete iwk;
  delete siwr;

  return 0;
};