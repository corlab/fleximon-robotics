#!/usr/bin/env python

import roslib
import rospy
import copy
import math

# smach
import smach
import smach_ros

from geometry_msgs.msg import Pose
from copy import deepcopy

import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import time

import tf
from tf.transformations import *
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

# Messages
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool

# starting other node
import roslaunch 
from roslaunch import ROSLaunch
import subprocess

### Create a handle for the Move Group Commander
mgc = MoveGroupCommander("ur5_manipulator")
ikf = MoveGroupCommander("ur5_manipulator")
plannerID = 'PRMstarkConfigDefault'
#plannerID = ''
goalPosTol = 0.001
goalOrientTol = 0.01
# RRTkConfigDefault
#mgc = MoveGroupCommander("ur5_ikfast")
#ikf = MoveGroupCommander("ur5_ikfast")

###############################################################################
class move_to_init(smach.StateMachine):
	clear = False

	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

	def execute(self, userdata):
		rospy.sleep(2.0)
		print "<==== Move To Init ====>"

		# Allow replanning to increase the odds of a solution
		mgc.allow_replanning(True)

		# set Planning Time
		mgc.set_planning_time(5.0)

		# set Planner
		mgc.set_planner_id(plannerID)

    # Set the right arm reference frame
		mgc.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		mgc.set_goal_position_tolerance(goalPosTol)
		mgc.set_goal_orientation_tolerance(goalOrientTol)
		
		# Get the name of the end-effector link
		end_effector_link = mgc.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = mgc.get_current_pose(end_effector_link).pose

		# Q1 Joint values in Order ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint','arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
		Q1 = [0.785,-1.57,1.57,-1.57,-1.57,0.785]	# init: Base = -173 Grad auf Bedienpanel

		# Set Q1 target joint value
		target_joint_value = mgc.set_joint_value_target(Q1) 	
      
		# Set the target of the group and then move the group to the specified target
		rospy.sleep(1.0)		
		mgc.go(target_joint_value)
		rospy.sleep(1.5)
		return 'succeeded'

###############################################################################
class move_to_magazin(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Move To Magazin ====>"
	# Initialize the ROS node
#		rospy.sleep(2.0)
		# Allow replanning to increase the odds of a solution
		ikf.allow_replanning(True)

		# set Planning Time
		ikf.set_planning_time(5.0)

		# set Planner
		ikf.set_planner_id(plannerID)

    # Set the right arm reference frame
		ikf.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		ikf.set_goal_position_tolerance(goalPosTol)
		ikf.set_goal_orientation_tolerance(goalOrientTol)
		
		# Get the name of the end-effector link
		end_effector_link = ikf.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = ikf.get_current_pose(end_effector_link).pose

		######
		listener = tf.TransformListener()
		count=0; 
 		flag = False
		while flag == False:
			try:
				(trans,rot) = listener.lookupTransform('/base_link','/grasp_frame', rospy.Time(0))
				flag = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue

		######
		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base_link'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = trans[0] - 0.0045
		target_pose.pose.position.y = 0.40941
		target_pose.pose.position.z = 0.45481
		target_pose.pose.orientation.x = start_pose.orientation.x
		target_pose.pose.orientation.y = start_pose.orientation.y
		target_pose.pose.orientation.z = start_pose.orientation.z 
		target_pose.pose.orientation.w = start_pose.orientation.w

		# not required at the moment 
		target_position = [target_pose.pose.position.x,
									 		 target_pose.pose.position.y, 
											 target_pose.pose.position.z]
		target_orientation = [target_pose.pose.orientation.x,
													target_pose.pose.orientation.y, 
													target_pose.pose.orientation.z,
													target_pose.pose.orientation.w]  
        
    # Set the internal state to the current state
#		mgc.set_start_state_to_current_state()

		# Set the target of the group and then move the group to the specified target
### Returns a dirty pathplanning with wrong orientation ###
#		target = mgc.set_pose_target(target_pose, end_effector_link)		
#		mgc.go(target)
#		rospy.sleep(2.0)

		# Return a motion plan (a RobotTrajectory) to the set goal state
		plan = ikf.plan(target_pose)
		# Execute a previously planned path
		rospy.sleep(1.0)
		ikf.execute(plan)
		rospy.sleep(1.5)
		return 'succeeded'

###############################################################################
class move_to_pregrasp(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Move To PreGrasp ====>"
	# Initialize the ROS node
#		rospy.sleep(2.0)
		# Allow replanning to increase the odds of a solution
		ikf.allow_replanning(True)

		# set Planning Time
		ikf.set_planning_time(5.0)

		# set Planner
		ikf.set_planner_id(plannerID)

    # Set the right arm reference frame
		ikf.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		ikf.set_goal_position_tolerance(goalPosTol)
		ikf.set_goal_orientation_tolerance(goalOrientTol)
		
		# Get the name of the end-effector link
		end_effector_link = ikf.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = ikf.get_current_pose(end_effector_link).pose
		
		listener = tf.TransformListener()
		count=0; 
 		flag = False
		while flag == False:
			try:
				(trans,rot) = listener.lookupTransform('/base_link','/grasp_frame', rospy.Time(0))
				flag = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue

		######
		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base_link'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = start_pose.position.x
		target_pose.pose.position.y = start_pose.position.y
		target_pose.pose.position.z = start_pose.position.z
		target_pose.pose.orientation.x = start_pose.orientation.x
		target_pose.pose.orientation.y = start_pose.orientation.y
		target_pose.pose.orientation.z = start_pose.orientation.z 
		target_pose.pose.orientation.w = start_pose.orientation.w



		# not required at the moment 
		target_position = [target_pose.pose.position.x,
									 		 target_pose.pose.position.y, 
											 target_pose.pose.position.z]
		target_orientation = [target_pose.pose.orientation.x,
													target_pose.pose.orientation.y, 
													target_pose.pose.orientation.z,
													target_pose.pose.orientation.w]  
        
    # Set the internal state to the current state
#		mgc.set_start_state_to_current_state()

		# Set the target of the group and then move the group to the specified target
### Returns a dirty pathplanning with wrong orientation ###
#		target = mgc.set_pose_target(target_pose, end_effector_link)		
#		mgc.go(target)
#		rospy.sleep(2.0)

		# Return a motion plan (a RobotTrajectory) to the set goal state
		plan = ikf.plan(target_pose)
		# Execute a previously planned path
		rospy.sleep(1.0)
		ikf.execute(plan)
		rospy.sleep(1.5)
		return 'succeeded'

###############################################################################
class rotate_in_pregrasp(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Rotate in Pregrasp ====>"
	# Initialize the ROS node
#		rospy.sleep(2.0)
		# Allow replanning to increase the odds of a solution
		ikf.allow_replanning(True)

		# set Planning Time
		ikf.set_planning_time(5.0)

		# set Planner
		ikf.set_planner_id(plannerID)

    # Set the right arm reference frame
		ikf.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		ikf.set_goal_position_tolerance(goalPosTol)
		ikf.set_goal_orientation_tolerance(goalOrientTol)
		
		# Get the name of the end-effector link
		end_effector_link = ikf.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = ikf.get_current_pose(end_effector_link).pose

		listener = tf.TransformListener()
		count=0; 
 		flag = False
		while flag == False:
			try:
				(trans,rot) = listener.lookupTransform('/base_link','/grasp_frame', rospy.Time(0))				
				flag = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue

		######
		print("#### Transform ####")
		rospy.loginfo("trans x: %s",trans[0])
		rospy.loginfo("trans y: %s",trans[1])
		rospy.loginfo("trans z: %s",trans[2])
		rospy.loginfo("rot x: %s",rot[0])
		rospy.loginfo("rot y: %s",rot[1])
		rospy.loginfo("rot z: %s",rot[2])
		rospy.loginfo("rot w: %s",rot[3])

		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base_link'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = start_pose.position.x
		target_pose.pose.position.y = start_pose.position.y
		target_pose.pose.position.z = start_pose.position.z
		target_pose.pose.orientation.x = rot[0]
		target_pose.pose.orientation.y = rot[1]
		target_pose.pose.orientation.z = rot[2]
		target_pose.pose.orientation.w = rot[3]
		rospy.loginfo("start_pose.orientation.x: %s",start_pose.orientation.x)
		rospy.loginfo("start_pose.orientation.y: %s",start_pose.orientation.y)
		rospy.loginfo("start_pose.orientation.z: %s",start_pose.orientation.z)
		rospy.loginfo("start_pose.orientation.w: %s",start_pose.orientation.w)



		# not required at the moment 
		target_position = [target_pose.pose.position.x,
									 		 target_pose.pose.position.y, 
											 target_pose.pose.position.z]
		target_orientation = [target_pose.pose.orientation.x,
													target_pose.pose.orientation.y, 
													target_pose.pose.orientation.z,
													target_pose.pose.orientation.w]  
        
    # Set the internal state to the current state
#		mgc.set_start_state_to_current_state()

		# Set the target of the group and then move the group to the specified target
### Returns a dirty pathplanning with wrong orientation ###
#		target = mgc.set_pose_target(target_pose, end_effector_link)		
#		mgc.go(target)
#		rospy.sleep(2.0)

		# Return a motion plan (a RobotTrajectory) to the set goal state
		plan = ikf.plan(target_pose)
		# Execute a previously planned path
		rospy.sleep(1.0)
		ikf.execute(plan)
		rospy.sleep(1.5)
		return 'succeeded'

###############################################################################

class move_ee_orientation(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Move EE Orientation ====>"

		# Allow replanning to increase the odds of a solution
		mgc.allow_replanning(True)

		# set Planning Time
		mgc.set_planning_time(5.0)

		# set Planner
		mgc.set_planner_id('PRMstarkConfigDefault')

    # Set the right arm reference frame
		mgc.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		mgc.set_goal_position_tolerance(0.01)
		mgc.set_goal_orientation_tolerance(0.1)
		
		# Get the name of the end-effector link
		end_effector_link = mgc.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = mgc.get_current_pose(end_effector_link).pose

		# Get and print joint values of current pose
		joint_values = mgc.get_current_joint_values()
		#print "Current joint values: %s" % (str(joint_values))
		print "Joint State1: %s" % (str(joint_values[5]))
#		print "Box Winkel: %s" % (str(get_coordinates.oz))
		wrist_3_joint = (get_coordinates.oz + joint_values[5] )
		# Q1 Joint values in Order ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint','arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
		Q1 = [joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],wrist_3_joint] 	# cam init 2

		# Set Q1 target joint value
		target_joint_value = mgc.set_joint_value_target(Q1) 	
      
		# Set the target of the group and then move the group to the specified target
		mgc.go(target_joint_value)

		rospy.sleep(2.5)
		return 'succeeded'
