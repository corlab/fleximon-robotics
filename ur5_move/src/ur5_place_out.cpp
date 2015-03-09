#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <math.h>

#define PI 3.14159265
#define GRIPPERdist 0.256
#define goalPosTol 0.001
#define goalOrientTol 0.05

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_manipulation");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // connecting to move group
  move_group_interface::MoveGroup group("ur5_manipulator");

  // set planner from OMPL lib
//  group.setPlannerId("PRMkConfigDefault");

  // print information about the endeffector
  std::string ee = group.getEndEffectorLink();
  ROS_INFO("Endeffector Frame %s",ee.c_str());
  ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

  // allow replanning to increase the odds of a solution
  group.allowReplanning(true);

  // set the refrance frame
  group.setPoseReferenceFrame("base_link");

  // allow some position (meters) and orientation (radians) tolerances
  group.setGoalPositionTolerance(goalPosTol);
  group.setGoalOrientationTolerance(goalOrientTol);

  // get the name of the end-effector link
  std::string end_effector_link = group.getEndEffectorLink();

  // start in the "init" configuration stored in the SRDF file
//  group.setNamedTarget("init2");

  // plan and execute a trajectory to the goal configuration

  // get the current pose so we can add it as a waypoint
  geometry_msgs::PoseStamped start_pose;
  start_pose = group.getCurrentPose(end_effector_link);
  ROS_INFO_STREAM("Start Pose: " << std::endl << start_pose);

  //  Initialize the waypoints list
  std::vector<geometry_msgs::Pose> waypoints;

  // set the first waypoint to be the starting pose
  geometry_msgs::Pose ROBOT_START_POSE;
  ROBOT_START_POSE.position.x = start_pose.pose.position.x;
  ROBOT_START_POSE.position.y = start_pose.pose.position.y;
  ROBOT_START_POSE.position.z = start_pose.pose.position.z;
  ROBOT_START_POSE.orientation.x = start_pose.pose.orientation.x;
  ROBOT_START_POSE.orientation.y = start_pose.pose.orientation.y;
  ROBOT_START_POSE.orientation.z = start_pose.pose.orientation.z;
  ROBOT_START_POSE.orientation.w = start_pose.pose.orientation.w;
  waypoints.push_back(ROBOT_START_POSE);

  // set the next waypoint
  geometry_msgs::Pose cartesian_target;

  cartesian_target.position.x = start_pose.pose.position.x;
  cartesian_target.position.y = start_pose.pose.position.y ;
  cartesian_target.position.z = start_pose.pose.position.z + 0.2;
  cartesian_target.orientation.x = start_pose.pose.orientation.x;
  cartesian_target.orientation.y = start_pose.pose.orientation.y;
  cartesian_target.orientation.z = start_pose.pose.orientation.z;
  cartesian_target.orientation.w = start_pose.pose.orientation.w;
  waypoints.push_back(cartesian_target);

  // plan the Cartesian path connecting the waypoints
  moveit_msgs::ExecuteKnownTrajectory srv;
  move_group_interface::MoveGroup::Plan plan;
  group.setPlanningTime(5);

  double fraction = group.computeCartesianPath(waypoints,
                                                   0.01, // eef_step
                                                   0.0, // jump_threshold
                                                   srv.request.trajectory, true);

      std::cout << "fraction: " << fraction << std::endl;

      robot_trajectory::RobotTrajectory rt (group.getCurrentState()->getRobotModel(), "ur5_manipulator");
      // Second get a RobotTrajectory from trajectory
      rt.setRobotTrajectoryMsg(*group.getCurrentState(), srv.request.trajectory);

      ROS_INFO_STREAM("Pose reference frame: " << group.getPoseReferenceFrame ());

      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      bool success = iptp.computeTimeStamps(rt);
      ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

      // Get RobotTrajectory_msg from RobotTrajectory
      rt.getRobotTrajectoryMsg(srv.request.trajectory);
      // Finally plan and execute the trajectory
      plan.trajectory_ = srv.request.trajectory;
      ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);

      if (fraction == 1.0) {
           group.execute(plan);
      } else
      ROS_WARN("Could not compute the cartesian path :( ");

      // ##### Execution
//      group.asyncExecute(plan);
//      group.execute(plan);


//      ros::NodeHandle node_handle;
//      ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");


     return 0;
}

