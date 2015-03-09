/*
 * ur5_move.cpp
 *
 *  Created on: 23.02.2015
 *      Author: M. Wojtynek
 */
#include "ur5_move.h"

// Constructor
UR5Move::UR5Move()
{

    referenceFrame = "/base_link";
    setPlanner ="PRMkConfigDefault";
    destinationFrame = "/base_link";
    graspFrame = "/grasp_frame";

    state = 11;
}

void UR5Move::mainNodeLoop()
{
    ros::Rate loop_rate(5);

    while(ros::ok())
    {
        switch (state)
        {
        case 0: // Move To Init
                moveInit();

                ROS_INFO("######### Move to Init! #########");
                state++;

            break;
        case 1: //
                ROS_INFO("######### Move to Magazine! ######### ");
                moveMagazine();
                state++;

            break;
        case 2: //
                ROS_INFO("######### Rotate EndEffector! ######### ");
                rotateEndEffector();
                state++;

            break;
        case 3: //
                ROS_INFO("######### Move In to Grasp Position! ######### ");
                moveIn();
                state++;

            break;
        case 4: //
                ROS_INFO("######### Move Precise! ######### ");
                moveJointPrecise();
                ros::shutdown();

            break;
        case 11: // Reset-Zustand
//            driveManually2Start = false;
//            driveManually2End = false;
            state = 0;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


}



void UR5Move::moveInit()
{
    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
    group.setPlannerId(setPlanner);

    // print information about the endeffector
    std::string ee = group.getEndEffectorLink();
    ROS_INFO("Endeffector Frame %s",ee.c_str());
    ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

    // allow replanning to increase the odds of a solution
    group.allowReplanning(true);
    group.setPlanningTime(5);
    // set the refrance frame
    group.setPoseReferenceFrame(referenceFrame);

    // allow some position (meters) and orientation (radians) tolerances
    group.setGoalPositionTolerance(goalPosTol);
    group.setGoalOrientationTolerance(goalOrientTol);

    group.setJointValueTarget("arm_shoulder_pan_joint", 0.785);
    group.setJointValueTarget("arm_shoulder_lift_joint", -1.57);
    group.setJointValueTarget("arm_elbow_joint", 1.57);
    group.setJointValueTarget("arm_wrist_1_joint", -1.57);
    group.setJointValueTarget("arm_wrist_2_joint", -1.57);
    group.setJointValueTarget("arm_wrist_3_joint", 0.785);
    // Joint value target execution
    group.move();
}

void UR5Move::moveJointPregrasp()
{
    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
    group.setPlannerId(setPlanner);

    // print information about the endeffector
    std::string ee = group.getEndEffectorLink();
    ROS_INFO("Endeffector Frame %s",ee.c_str());
    ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

    // allow replanning to increase the odds of a solution
    group.allowReplanning(true);
    group.setPlanningTime(5);
    // set the refrance frame
    group.setPoseReferenceFrame(referenceFrame);

    // allow some position (meters) and orientation (radians) tolerances
    group.setGoalPositionTolerance(goalPosTol);
    group.setGoalOrientationTolerance(goalOrientTol);

    // get the name of the end-effector link
    std::string end_effector_link = group.getEndEffectorLink();

    group.setJointValueTarget("arm_shoulder_pan_joint", 0.3295);
    group.setJointValueTarget("arm_shoulder_lift_joint", -1.7383);
    group.setJointValueTarget("arm_elbow_joint", 1.758);
    group.setJointValueTarget("arm_wrist_1_joint", -1.7924);
    group.setJointValueTarget("arm_wrist_2_joint", -1.9579);
    group.setJointValueTarget("arm_wrist_3_joint", 0.311);

    // Joint value target execution
    group.move();
}

void UR5Move::moveJointGrasp()
{
    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
    group.setPlannerId(setPlanner);

    // print information about the endeffector
    std::string ee = group.getEndEffectorLink();
    ROS_INFO("Endeffector Frame %s",ee.c_str());
    ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

    // allow replanning to increase the odds of a solution
    group.allowReplanning(true);
    group.setPlanningTime(5);
    // set the refrance frame
    group.setPoseReferenceFrame(referenceFrame);

    // allow some position (meters) and orientation (radians) tolerances
    group.setGoalPositionTolerance(goalPosTol);
    group.setGoalOrientationTolerance(goalOrientTol);

    // get the name of the end-effector link
    std::string end_effector_link = group.getEndEffectorLink();

    group.setJointValueTarget("arm_shoulder_pan_joint", 0.1177);
    group.setJointValueTarget("arm_shoulder_lift_joint", -1.4437);
    group.setJointValueTarget("arm_elbow_joint", 2.129);
    group.setJointValueTarget("arm_wrist_1_joint", -2.5362);
    group.setJointValueTarget("arm_wrist_2_joint", -1.9072);
    group.setJointValueTarget("arm_wrist_3_joint", 0.091);

    // Joint value target execution
    group.move();
}

void UR5Move::moveMagazine(){

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
//    group.setPlannerId(setPlanner);

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

    // Transform listener
      try {
          listener.waitForTransform(destinationFrame, graspFrame, ros::Time(0), ros::Duration(5.0));
          listener.lookupTransform(destinationFrame, graspFrame, ros::Time(0), transform);

//          listener.waitForTransform(destinationFrame, graspFrame, ros::Time(0), ros::Duration(5.0));
//          listener.transformPoint(destinationFrame, graspFrame,transOut);
      } catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }

    std::cout << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << "," << std::endl;


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
    cartesian_target.position.x = transform.getOrigin().x() - 0.0045;
    cartesian_target.position.y = 0.40931;
    cartesian_target.position.z = 0.45486;
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

}

void UR5Move::rotateEndEffector(){

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
//    group.setPlannerId(setPlanner);

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

    // Transform listener
      try {
          listener.waitForTransform(destinationFrame, graspFrame, ros::Time(0), ros::Duration(5.0));
          listener.lookupTransform(destinationFrame, graspFrame, ros::Time(0), transform);

//          listener.waitForTransform(destinationFrame, graspFrame, ros::Time(0), ros::Duration(5.0));
//          listener.transformPoint(destinationFrame, graspFrame,transOut);
      } catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }

    std::cout << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << "," << std::endl;


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
    cartesian_target.position.y = start_pose.pose.position.y;
    cartesian_target.position.z = start_pose.pose.position.z;
    cartesian_target.orientation.x = transform.getRotation().getX();
    cartesian_target.orientation.y = transform.getRotation().getY();
    cartesian_target.orientation.z = transform.getRotation().getZ();
    cartesian_target.orientation.w = transform.getRotation().getW();
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

}


void UR5Move::moveIn(){

    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    // set planner from OMPL lib
//    group.setPlannerId("PRMkConfigDefault");

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

    double dist_y = 0;
    double dist_z = 0;
    double angle = 25;

    dist_y = GRIPPERdist * sin(angle * PI / 180);
    dist_z = GRIPPERdist * cos(angle * PI / 180);
    std::cout << "dist_y: " << dist_y << std::endl;
    std::cout << "dist_z: " << dist_z << std::endl;


    cartesian_target.position.x = start_pose.pose.position.x;
    cartesian_target.position.y = start_pose.pose.position.y - dist_y;
    cartesian_target.position.z = start_pose.pose.position.z - dist_z;
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

        std::vector<double> joint_values = group.getCurrentJointValues();
        std::cout << "Joint Values: " << joint_values[0] << std::endl;
        std::cout << "Joint Values: " << joint_values[1] << std::endl;
        std::cout << "Joint Values: " << joint_values[2] << std::endl;
        std::cout << "Joint Values: " << joint_values[3] << std::endl;
        std::cout << "Joint Values: " << joint_values[4] << std::endl;
        std::cout << "Joint Values: " << joint_values[5] << std::endl;



}

void UR5Move::moveJointPrecise()
{
    // connecting to move group
    move_group_interface::MoveGroup group("ur5_manipulator");

    std::vector<double> joint_values = group.getCurrentJointValues();
    std::cout << "Joint Values: " << joint_values[0] << std::endl;
    std::cout << "Joint Values: " << joint_values[1] << std::endl;
    std::cout << "Joint Values: " << joint_values[2] << std::endl;
    std::cout << "Joint Values: " << joint_values[3] << std::endl;
    std::cout << "Joint Values: " << joint_values[4] << std::endl;
    std::cout << "Joint Values: " << joint_values[5] << std::endl;



    // set planner from OMPL lib
    group.setPlannerId(setPlanner);

    // print information about the endeffector
    std::string ee = group.getEndEffectorLink();
    ROS_INFO("Endeffector Frame %s",ee.c_str());
    ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

    // allow replanning to increase the odds of a solution
    group.allowReplanning(true);
    group.setPlanningTime(5);
    // set the refrance frame
    group.setPoseReferenceFrame(referenceFrame);

    // allow some position (meters) and orientation (radians) tolerances
    group.setGoalPositionTolerance(goalPosTol);
    group.setGoalOrientationTolerance(goalOrientTol);

    // set Joint Values
    group.setJointValueTarget("arm_shoulder_pan_joint", 0.1181);
    group.setJointValueTarget("arm_shoulder_lift_joint", -1.444);
    group.setJointValueTarget("arm_elbow_joint", 2.1294);
    group.setJointValueTarget("arm_wrist_1_joint", -2.5372);
    group.setJointValueTarget("arm_wrist_2_joint", -1.9083);
    group.setJointValueTarget("arm_wrist_3_joint", 0.0913);
    // Joint value target execution
    group.move();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_move");

    UR5Move instance;
    instance.mainNodeLoop();
    return 0;
}
