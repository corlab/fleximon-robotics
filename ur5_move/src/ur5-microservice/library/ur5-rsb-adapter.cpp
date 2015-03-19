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
             std::cout << "### Path Planning & Execution finished ###" << std::endl;
        } else
        ROS_WARN("Could not compute the cartesian path :( ");

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
