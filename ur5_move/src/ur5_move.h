/*
 * ur5_move.cpp
 *
 *  Created on: 23.02.2015
 *      Author: M. Wojtynek
 */

#ifndef UR5Move_H
#define UR5Move_H

#define RESET 11 // Reset-Zustand
#define MAX_TP 3 // maximal zulaessiger Abstand zum Tisch (oberer Rand)
#define NEXT_TP 0.20 // Abstand zwischen den Positionen vor dem Tisch
#define TABLE_DIS 0.97 // Bis zu dieser Entfernung faehrt der Youbot an den Tisch
#define DRIVE_OFF_DIS 0.70 // Diese Strecke faehrt der Youbot rueckwaerts um nicht anzustossen

#define goalPosTol 0.005
#define goalOrientTol 0.05

//#define goalPosTol 0.001
//#define goalOrientTol 0.0001

// math
#define PI 3.14159265

// Gripper
#define GRIPPERdist 0.256
#define GP_DIS 0.256 // Distance Gripper - Target
#define GP_LEN 0.104 // Length ur5 arm_ee joint - Gripper Fingers

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



class UR5Move
{
public:
    UR5Move();
    void mainNodeLoop();
private:  
    ros::NodeHandle nH;
    ros::Publisher statusPub, velPub;
    ros::Subscriber gripStatusSub, diffPosSub, nextStepSub, driveSub, scanSub;
    ros::ServiceClient clientGrip, clientDrop, clientCancel;
    ros::ServiceServer serverC;
    ros::Timer timer;

    int state;
    std::string objectName;
    std::string containerName;
    int currTP;
    double midDistance;
    bool driveManually2Start;
    bool driveManually2End;
    bool nextStep;
    bool gotCmd;
    bool gotGripStatus;
    bool gotObject;
    bool noObject;
    bool gotPoint;
    bool gotDropStatus;
    bool gotBox;
    bool noBox;
    bool timeout;
    bool usedDiffPoint;

    geometry_msgs::Pose startPos;
    geometry_msgs::Point diffPos;
    geometry_msgs::Pose endPos;
    geometry_msgs::Pose tablePos[MAX_TP];
    std::string referenceFrame,setPlanner,destinationFrame,graspFrame;

    // Transform
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::PointStamped transOut;

    void moveInit();
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
