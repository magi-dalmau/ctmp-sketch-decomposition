#include <brfs.hpp>
#include <iostream>
#include <iwk.hpp>
#include <moveit_tamp_problem.hpp>
#include <random>
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "test_tiago_problem_node");
  ROS_INFO("Starting Tiago problem test");
  // Declare Node Handle
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // ROS_INFO("TESTING HASH");
  // std::size_t hash = 0;
  // std::vector<std::size_t> features = {15, 24, 60};

  // for (const auto f : features) {
  //   test_hash_combine<std::size_t>(hash, f);
  // }
  // std::cout << "1st hash: " << hash << std::endl;

  // hash = 0;
  //  features = {15,24, 60};
  // for (const auto f : features) {
  //   test_hash_combine<std::size_t>(hash, f);
  // }
  // std::cout << "2nd hash: " << hash << std::endl;

  // hash = 0;
  //  features = {24, 10, 65};
  // for (const auto f : features) {
  //   test_hash_combine<std::size_t>(hash, f);
  // }
  // std::cout << "3d hash: " << hash << std::endl;
  // return 0;

  bool lazy = nh.param("lazy", true);
  std::string planning_group = nh.param("planning_group", std::string("arm_torso"));

  // moveit::planning_interface::MoveGroupInterface move_group_interface(planning_group);

  std::cout << "The problem will be initialized" << std::endl;
  std::string problem_name = nh.param("problem_name", std::string("pb_3"));
  const std::string filename =
      "/ros_ws/src/ros-tamp/benchmarkings/lagriffoul/problems/pb_3_sorting_objects/problem_definitions/" +
      problem_name + ".xml";

  auto problem = new MoveitTampProblem(filename, planning_group, &nh);

  std::cout << "My problem is:\n" << *problem << std::endl;

  // auto brfs = new BrFS(problem);

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

  auto iwk = new IWk(problem, nh.param("k_max", 1));

  Plan plan;
  if (iwk->Solve(plan, lazy)) {
    std::cout << "Found solution" << std::endl;
    std::cout << plan << std::endl;
    while (ros::ok()) {
      problem->ExecutePlan(plan);
    }
  } else {
    std::cout << "Solution not found" << std::endl;
  }

  // problem->checkIk();
  // problem->isValidConfig();
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // problem->planMovement();
  // problem->addTestCollision();
  // if (problem->planMovement(plan)) {
  //   problem->executePlan(plan);

  // } else {
  //   std::cout<<"There was an object colliding, i will move it"<<std::endl;
  //   problem->moveTestCollision();
  //   if (problem->planMovement(plan))
  //     problem->executePlan(plan);
  // }
  // std::cout << "My problem is:\n" << *problem << std::endl;

  // auto brfs = new BrFS(problem);

  // Plan plan;
  // if (brfs->solve(plan, lazy)) {
  //   std::cout << "Found solution" << std::endl;
  //   std::cout << plan << std::endl;
  // } else {
  //   std::cout << "Solution not found" << std::endl;
  // }
  ros::waitForShutdown();
  delete problem;
  delete iwk;

  return 0;
};