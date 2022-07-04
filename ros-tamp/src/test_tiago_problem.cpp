#include <brfs.hpp>
#include <cluttered_tamp_problem.hpp>
#include <non_monotonic_tamp_problem.hpp>
#include <iostream>
#include <iwk.hpp>
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

  Problem *problem;
  if (problem_name.find("3") != std::string::npos) {
    const std::string filename =
        "/ros_ws/src/ros-tamp/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/problem_definitions/" +
        problem_name + ".xml";
    problem = new ClutteredTampProblem(filename, planning_group, &nh);

  } else if (problem_name.find("4") != std::string::npos) {
    std::cout<<"SELECTED PROBLEM TYPE 4"<<std::endl;
    const std::string filename =
        "/ros_ws/src/ros-tamp/benchmarkings/lagriffoul/problems/pb_4_non_monotonic/problem_definitions/" +
        problem_name + ".xml";
    problem = new NonMonotonicTampProblem(filename, planning_group, &nh);
  } else {
    std::cout << "Unknown  problem type for name: " << problem_name << std::endl;
    return 0;
  }

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
    //std::cout << plan << std::endl;
    std::cout << "Total cost is " << plan.total_cost << std::endl;
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