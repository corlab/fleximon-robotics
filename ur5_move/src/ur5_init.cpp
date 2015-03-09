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
#define GRIPPERdist 0.002
#define goalPosTol 0.001
#define goalOrientTol 0.01

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
  std::string setPlanner ="PRMkConfigDefault";
  group.setPlannerId(setPlanner);

  // print information about the endeffector
  std::string ee = group.getEndEffectorLink();
  ROS_INFO("Endeffector Frame %s",ee.c_str());
  ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

  // allow replanning to increase the odds of a solution
  group.allowReplanning(true);
  group.setPlanningTime(5);
  // set the refrance frame
  std::string referenceFrame = "/base_link";
  group.setPoseReferenceFrame(referenceFrame);

  // allow some position (meters) and orientation (radians) tolerances
  group.setGoalPositionTolerance(goalPosTol);
  group.setGoalOrientationTolerance(goalOrientTol);

  // get the name of the end-effector link
  std::string end_effector_link = group.getEndEffectorLink();

  group.setJointValueTarget("arm_shoulder_pan_joint", 0.785);
  group.setJointValueTarget("arm_shoulder_lift_joint", -1.57);
  group.setJointValueTarget("arm_elbow_joint", 1.57);
  group.setJointValueTarget("arm_wrist_1_joint", -1.57);
  group.setJointValueTarget("arm_wrist_2_joint", -1.57);
  group.setJointValueTarget("arm_wrist_3_joint", 0.785);

  // Joint value target execution
  group.move();







  return 0;
}


