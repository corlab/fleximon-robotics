/*
 * ur5_move.cpp
 *
 *  Created on: 23.02.2015
 *      Author: M. Wojtynek
 */

#ifndef UR5Move_H
#define UR5Move_H

#define goalPosTol 0.005
#define goalOrientTol 0.05

//#define goalPosTol 0.001
//#define goalOrientTol 0.0001

// math
#define PI 3.14159265

// Gripper
#define GP_DIS 0.15 // Distance Gripper - Target, Grip = 0.256
#define GP_LEN 0.104 // Length ur5 arm_ee joint - Gripper Fingers

// Anfahren Magazin
#define MG_Z 0.25       // Z-Offset an Magazin
#define MG_ANGLE 25     // Neigungswinkel Magazin in Grad

// Werkstuecktraeger - Charge Carrier
#define WT_X 0.67375
#define WT_Y 0.83677
#define WT_Z 0.18433

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "trajectory_msgs/JointTrajectoryPoint.h"

// Transform
#include <tf/transform_listener.h>

#include <string.h>



class UR5Move
{
public:
    UR5Move();
    void mainNodeLoop();
private:  
    ros::NodeHandle nH;
    ros::Timer timer;

    int state;

    std::string referenceFrame,setPlanner,destinationFrame,graspFrame;


    // Transform
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::PointStamped transOut;

    void moveInit();
    void moveInit2();
    void moveMagazine();
    void rotateEndEffector();
    void moveJointPregrasp();
    void moveJointGrasp();
    void moveIn();
    void moveJointPrecise();
    void moveWT();
    void moveOut();


};

#endif // UR5Move_H
