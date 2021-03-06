/* ============================================================
 *
 * This file is a part of the ur5-microservice project
 *
 * Copyright (C) 2015 by Sebastian Wrede <swrede at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#include "ur5-rsb-adapter.h"

#include <rsb/Factory.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/kinematics/JointAngles.pb.h>

#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <math.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
// Transform
#include <tf/transform_listener.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ur_msgs/SetTeachIn.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "trajectory_msgs/JointTrajectoryPoint.h"



// Tolerances
#define goalPosTol 0.001
#define goalOrientTol 0.01
// Anfahren Magazin
#define MG_Z 0.25       // Z-Offset an Magazin
#define MG_ANGLE 25     // Neigungswinkel Magazin in Grad
// math
#define PI 3.14159265


using namespace std;
using namespace rsb;
using namespace rsb::patterns;

namespace ur5_microservice {
namespace library {

class SetTeachInCallback: public LocalServer::Callback<bool, void> {
public:
    void call(const std::string& /*methodName*/, boost::shared_ptr<bool> input) {
        std::cout << "SetTeachIn method called..." << std::endl;
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<ur_msgs::SetTeachIn>("ur_driver/setTeachIn");
        ur_msgs::SetTeachIn srv;
        srv.request.active = *input;
        std::cout << "Call of SetTeachIn." << std::endl;
        client.call(srv);
        std::cout << "Reached end of SetTeachIn." << std::endl;
    }
};

class QueryPoseCallback: public LocalServer::Callback<void, rst::geometry::Pose> {
public:
    boost::shared_ptr<rst::geometry::Pose> call(const std::string& /*methodName*/) {
        std::cout << "QueryPose method called..." << std::endl;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
          listener.waitForTransform("/base_link", "/arm_afag_gripper_link", ros::Time(0), ros::Duration(5.0));
          listener.lookupTransform("/base_link", "/arm_afag_gripper_link", ros::Time(0), transform);
          boost::shared_ptr<rst::geometry::Pose> pose(new rst::geometry::Pose());
          pose->mutable_translation()->set_x(transform.getOrigin().x());
          pose->mutable_translation()->set_y(transform.getOrigin().y());
          pose->mutable_translation()->set_z(transform.getOrigin().z());
          pose->mutable_rotation()->set_qx(transform.getRotation().x());
          pose->mutable_rotation()->set_qy(transform.getRotation().y());
          pose->mutable_rotation()->set_qz(transform.getRotation().z());
          pose->mutable_rotation()->set_qw(transform.getRotation().w());
          std::cout << "QueryPose will return rst::geometry::pose: Translation (x,y,z): " << pose->translation().x() << "," << pose->translation().y() << "," << pose->translation().z() << " ";
          std::cout << " Rotation (qx,qy,qz,qw): " << pose->rotation().qx() << "," << pose->rotation().qy() << "," << pose->rotation().qz() << "," << pose->rotation().qw() << std::endl;
          return pose;
        } catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            throw;
        }
    }
};

/**
 * Move in Cartesian space towards a desired target position using
 * planning+IK features of respective backend.
 */

class MoveToCallback: public LocalServer::Callback<rst::geometry::Pose, void> {
public:
    void call(const std::string& /*methodName*/,
                                        boost::shared_ptr<rst::geometry::Pose> input) {
    	std::cout << "MoveCS method called with rst::geometry::pose: Translation (x,y,z): " << input->translation().x() << "," << input->translation().y() << "," << input->translation().z() << " ";
    	std::cout << " Rotation (qx,qy,qz,qw): " << input->rotation().qx() << "," << input->rotation().qy() << "," << input->rotation().qz() << "," << input->rotation().qw() << std::endl;
        move(input);
    }
    bool move(boost::shared_ptr<rst::geometry::Pose> input){

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
        cartesian_target.position.x = input->translation().x();
        cartesian_target.position.y = input->translation().y();
        cartesian_target.position.z = input->translation().z();
        cartesian_target.orientation.x = input->rotation().qx();
        cartesian_target.orientation.y = input->rotation().qy();
        cartesian_target.orientation.z = input->rotation().qz();
        cartesian_target.orientation.w = input->rotation().qw();
        waypoints.push_back(cartesian_target);

        // plan the Cartesian path connecting the waypoints
        moveit_msgs::ExecuteKnownTrajectory srv;
        move_group_interface::MoveGroup::Plan plan;
        group.setPlanningTime(5);

        double fraction = group.computeCartesianPath(waypoints,
                                                         0.001, // eef_step
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
             std::cout << "### Path Planning & Execution finished ###" << std::endl;
        } else
        ROS_WARN("Could not compute the cartesian path :( ");

        /*
        // start a background "spinner", so our node can process ROS messages
        //  - this lets us know when the move is completed
        ros::AsyncSpinner spinner(1);
        spinner.start();
        move_group_interface::MoveGroup group("ur5_manipulator");
        // set planner from OMPL lib
        group.setPlannerId("PRMkConfigDefault");

        // print information about the endeffector
        std::string ee = group.getEndEffectorLink();
        ROS_INFO("Endeffector Frame %s",ee.c_str());
        ROS_INFO_STREAM("Endeffector POSE" << std::endl << group.getCurrentPose(ee));

        // allow replanning to increase the odds of a solution
        group.allowReplanning(true);
        group.setPlanningTime(5);
        // set the refrance frame
        group.setPoseReferenceFrame("base_link");

        // allow some position (meters) and orientation (radians) tolerances
        group.setGoalPositionTolerance(goalPosTol);
        group.setGoalOrientationTolerance(goalOrientTol);

        // get the name of the end-effector link
        std::string end_effector_link = group.getEndEffectorLink();

        // plan and execute a trajectory to the goal configuration
        // get the current pose so we can add it as a waypoint
        geometry_msgs::PoseStamped start_pose;
        start_pose = group.getCurrentPose(end_effector_link);
        ROS_INFO_STREAM("Start Pose: " << std::endl << start_pose);
        robot_state::RobotState start_state(*group.getCurrentState());
        group.setStartState(start_state);

        // Planning to a Pose goal
        // ^^^^^^^^^^^^^^^^^^^^^^^
        // We can plan a motion for this group to a desired pose for the
        // end-effector.
        geometry_msgs::Pose target_pose1;

        target_pose1.position.x = input->translation().x();
        target_pose1.position.y = input->translation().y();
        target_pose1.position.z = input->translation().z();
        target_pose1.orientation.x = input->rotation().qx();
        target_pose1.orientation.y = input->rotation().qy();
        target_pose1.orientation.z = input->rotation().qz();
        target_pose1.orientation.w = input->rotation().qw();
        group.setPoseTarget(target_pose1);

        // Now, we call the planner to compute the plan
        // and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
//        group.move();
        moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success = group.plan(my_plan);
        group.execute(my_plan);
        */
        return success;

    }
};

/**
 * Move in joint space directly to a particular robot pose.
 */
class MoveCallback: public LocalServer::Callback<rst::kinematics::JointAngles, void> {
    void call(const std::string& /*methodName*/,
    		  boost::shared_ptr<rst::kinematics::JointAngles> input) {
        std::cout << "MoveJS method called with rst:kinematics:JointAngles vector in rad: ";
        double values[5];
        for (int i=0; i<input->angles_size(); i++) {
        	std::cout << " " << input->angles(i) << " ";
            values[i] = input->angles(i);
        }
        std::cout << std::endl;

        string referenceFrame = "/base_link";
        string setPlanner ="PRMkConfigDefault";
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
        robot_state::RobotState start_state(*group.getCurrentState());
        group.setStartState(start_state);

        // allow replanning to increase the odds of a solution
        group.allowReplanning(true);
        group.setPlanningTime(5);
        // set the refrance frame
        group.setPoseReferenceFrame(referenceFrame);

        // allow some position (meters) and orientation (radians) tolerances
        group.setGoalPositionTolerance(goalPosTol);
        group.setGoalOrientationTolerance(goalOrientTol);

        group.setJointValueTarget("arm_shoulder_pan_joint", values[0]);
        group.setJointValueTarget("arm_shoulder_lift_joint", values[1]);
        group.setJointValueTarget("arm_elbow_joint", values[2]);
        group.setJointValueTarget("arm_wrist_1_joint", values[3]);
        group.setJointValueTarget("arm_wrist_2_joint", values[4]);
        group.setJointValueTarget("arm_wrist_3_joint", values[5]);
        // Joint value target execution
        group.move();
        std::cout << "### Path Planning & Execution finished ###" << std::endl;

    }


};

UR5RSBAdapter::UR5RSBAdapter(const std::string& scope) {
    // Use the RSB factory to create a Server instance that provides
    // callable methods under the scope /example/server.
    Factory& factory = getFactory();
    server = factory.createLocalServer(scope);

    // Register method with name and implementing callback object.
    server->registerMethod("moverobot", LocalServer::CallbackPtr(new MoveCallback()));
    server->registerMethod("movetorobot", LocalServer::CallbackPtr(new MoveToCallback()));
    server->registerMethod("queryrobotpose", LocalServer::CallbackPtr(new QueryPoseCallback()));
    server->registerMethod("setteachin", LocalServer::CallbackPtr(new SetTeachInCallback()));
}

UR5RSBAdapter::~UR5RSBAdapter() {
}

/**
 * just used here as an example for GTest
 */
int UR5RSBAdapter::transform(const int& magicNumber) {
    return magicNumber * 2;
}

}
}
