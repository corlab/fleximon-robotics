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
#ikf = MoveGroupCommander("ur5_ikfast")
ikf = MoveGroupCommander("ur5_manipulator")

###############################################################################
class move_to_init_by_driver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

	def execute(self, userdata):
		
		rospy.sleep(2.0)
		client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		print "Waiting for server..."
		client.wait_for_server()
		print "Connected to server"	

		JOINT_NAMES = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
		'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']

		# 90 Grad Home		
#		Q1 = [1.0,-1.55,1.55,0,0,0]
		# Init
		Q1 = [-3.14,-2.356,-1.57,0.785,1.57,0]

		g = FollowJointTrajectoryGoal()
		g.trajectory = JointTrajectory()
		g.trajectory.joint_names = JOINT_NAMES
		g.trajectory.points = [
				JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
		
		client.send_goal(g)
		try:
				client.wait_for_result()
		except KeyboardInterrupt:
				client.cancel_goal()
				raise

		print "<==== Move To Init ====>"
		return 'succeeded'
###############################################################################

class get_coordinates(smach.State):
	x = -1
	y = -1
	z = -1
	ox	= -1
	oy	= -1
	oz  = -1
	ow	= -1
	updated = False

	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

	def callback_one(self, data):
#		rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.x)
#		rospy.loginfo("Boxkoordinate X: %f",data.point.x)
#		rospy.loginfo("Boxkoordinate Y: %f",data.point.y)
#		rospy.loginfo("Boxkoordinate Z: %f",data.point.z)	
		get_coordinates.x = data.point.x
		get_coordinates.y = data.point.y
		get_coordinates.z = data.point.z
#		rospy.loginfo("box_x: %s",box_x)
#		rospy.loginfo("box_y: %s",box_y)
#		rospy.loginfo("box_z: %s",box_z)
		# Get and print joint values of current pose
		joint_values = mgc.get_current_joint_values()			
		print "Joint State: %s" % (str(joint_values[5]))
		# unregister to avoid callback loop
		sub_one.unregister()
		get_coordinates.updated = True

	def callback_two(self, orientation):
#		rospy.loginfo("orientation X: %f",orientation.quaternion.x)
#		rospy.loginfo("orientation Y: %f",orientation.quaternion.y)
#		rospy.loginfo("orientation Z: %f",orientation.quaternion.z)
#		rospy.loginfo("orientation W: %f",orientation.quaternion.w)
#		euler = euler_from_quaternion([orientation.quaternion.x,orientation.quaternion.y,orientation.quaternion.z,orientation.quaternion.w])
#	#	rospy.loginfo("Euler R: %f", euler[0])
#		rospy.loginfo("Euler P: %f", euler[1])
#		rospy.loginfo("Euler Y: %f", euler[2])
#		get_coordinates.ox = euler[2]
		get_coordinates.oz = orientation.point.z
		rospy.loginfo("orientation Z: %f",orientation.point.z)
		# unregister to avoid callback loop
		sub_two.unregister()
		get_coordinates.updated = True


	def execute(self, userdata):
		print "<==== Get Coordinates and Orientation ====>"

		# Subscriber("Topic", Type, Function)
		global sub_one
		sub_one = rospy.Subscriber("perception/box_center", PointStamped, self.callback_one)
		global sub_two
		sub_two = rospy.Subscriber("perception/box_angle", PointStamped, self.callback_two)
		rospy.sleep(2.0)
		while not get_coordinates.updated:
			pass	
		get_coordinates.updated = False						
		return 'succeeded'

###############################################################################

class shoot_pic(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

	def execute(self, userdata):
		print "<==== Shoot Picture ====>"
		# Open image_view node with image_saver. Ajusted launch file with file saving path inside state_machine
		roslaunchProcess = subprocess.Popen(["roslaunch", "state_machine", "image_saver.launch"])
#		time.sleep(2.0)		
		roslaunchProcess.kill()
		return 'succeeded'

###############################################################################
class move_to_init(smach.StateMachine):
	clear = False

	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

	def execute(self, userdata):
		print "<==== Move To Init ====>"
#		rospy.sleep(5.0)

		# clear Octomap
		self.status_msg = Bool()
		self.status_msg.data = True

		# Allow replanning to increase the odds of a solution
		mgc.allow_replanning(True)

		# set Planning Time
		mgc.set_planning_time(5.0)

		# set Planner
#		mgc.set_planner_id('PRMstarkConfigDefault')

    # Set the right arm reference frame
		mgc.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		mgc.set_goal_position_tolerance(0.01)
		mgc.set_goal_orientation_tolerance(0.1)
		
		# Get the name of the end-effector link
		end_effector_link = mgc.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = mgc.get_current_pose(end_effector_link).pose

		# Q1 Joint values in Order ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint','arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
#		Q1 = [-3.14,-1.55,1.55,0,0,0] 					# 90 Grad home
		Q1 = [0.0,-0.785,0.785,-1.57,-1.57,0]
#		Q1 = [0.0,-0.785,1.57,-2.355,-1.57,0] 	# cam init 2
#		Q1 = [0.0,-0.785,0.785,-3.14,-1.57,0] 	# cam init 1
#		Q1 = [-3.14,-2.356,-1.57,-0.785,1.57,0]	# init

		# Set Q1 target joint value
		target_joint_value = mgc.set_joint_value_target(Q1) 	
      
		# Set the target of the group and then move the group to the specified target
		mgc.go(target_joint_value)

		rospy.sleep(2.0)
		return 'succeeded'
###############################################################################
class move_to_box(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Move To Box ====>"
	# Initialize the ROS node
#		rospy.sleep(5.0)
		# Allow replanning to increase the odds of a solution
		ikf.allow_replanning(True)

		# set Planning Time
		ikf.set_planning_time(5.0)

		# set Planner
		ikf.set_planner_id('RRTstarkConfigDefault')

    # Set the right arm reference frame
		ikf.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		ikf.set_goal_position_tolerance(0.01)
		ikf.set_goal_orientation_tolerance(0.1)
		
		# Get the name of the end-effector link
		end_effector_link = ikf.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = ikf.get_current_pose(end_effector_link).pose

		######
		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base_link'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = round(get_coordinates.x, 2)
		target_pose.pose.position.y = round(get_coordinates.y, 2) 
		target_pose.pose.position.z = round(get_coordinates.z, 2) + 0.55
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

		rospy.loginfo("box_bei_move_x: %s",round(get_coordinates.x, 2))
		rospy.loginfo("box_bei_move_y: %s",round(get_coordinates.y, 2))
		rospy.loginfo("box_bei_move_z: %s",get_coordinates.z)

		# Set the target of the group and then move the group to the specified target
### Returns a dirty pathplanning with wrong orientation ###
#		target = mgc.set_pose_target(target_pose, end_effector_link)		
#		mgc.go(target)
#		rospy.sleep(2.0)

		# Return a motion plan (a RobotTrajectory) to the set goal state
		plan = ikf.plan(target_pose)
		# Execute a previously planned path
		ikf.execute(plan)
		rospy.sleep(2.0)
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
#		mgc.set_planner_id('PRMstarkConfigDefault')

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

		rospy.sleep(2.0)
		return 'succeeded'

###############################################################################
class move_to_box_cartesian(smach.StateMachine):
	def __init__(self, x, y, z):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
	
	def execute(self, userdata):
		print "<==== Move To Box ====>"
	# Initialize the ROS node
		rospy.sleep(5.0)
		# Allow replanning to increase the odds of a solution
		mgc.allow_replanning(True)

		# set Planning Time
		mgc.set_planning_time(5.0)

		# set Planner
#		mgc.set_planner_id('PRMstarkConfigDefault')

    # Set the right arm reference frame
		mgc.set_pose_reference_frame('base_link')
		
		# Allow some leeway in position(meters) and orientation (radians)
		mgc.set_goal_position_tolerance(0.01)
		mgc.set_goal_orientation_tolerance(0.1)
		
		# Get the name of the end-effector link
		end_effector_link = mgc.get_end_effector_link()

    # Get the current pose so we can add it as a waypoint
		start_pose = mgc.get_current_pose(end_effector_link).pose

    # Initialize the waypoints list
		waypoints = []

    # Set the first waypoint to be the starting pose        
    # Append the start_pose to the waypoints list
		waypoints.append(start_pose)
		wpose = deepcopy(start_pose)

    # Set the next waypoint back 0.2 meters and right 0.2 meters
		wpose.position.x = round(get_coordinates.x, 2)
		wpose.position.y = round(get_coordinates.y, 2)
		wpose.position.z = round(get_coordinates.z, 2)
		wpose.orientation.x = start_pose.orientation.x
        
		waypoints.append(deepcopy(wpose))

		
		
		######
		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base_link'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = round(get_coordinates.x, 2)	
		target_pose.pose.position.y = round(get_coordinates.y, 2)
		target_pose.pose.position.z = start_pose.position.z
		target_pose.pose.orientation.x = start_pose.orientation.x
		target_pose.pose.orientation.y = start_pose.orientation.y 
		target_pose.pose.orientation.z = start_pose.orientation.z
		target_pose.pose.orientation.w = start_pose.orientation.w

		target_position = [target_pose.pose.position.x,
									 		 target_pose.pose.position.y, 
											 target_pose.pose.position.z]
		target_orientation = [target_pose.pose.orientation.x,
													target_pose.pose.orientation.y, 
													target_pose.pose.orientation.z,
													target_pose.pose.orientation.w]  
        
    # Set the internal state to the current state
		mgc.set_start_state_to_current_state()

#		mgc.set_position_target(target_position, end_effector_link)		
      
		(plan, fraction) = mgc.compute_cartesian_path (
								waypoints,   # waypoint poses
								0.01,		# eef_step
								0.0,		 # jump_threshold
								True)		# avoid_collisions

#		if len(plan.joint_trajectory.points) == 0:
#			return 'failed'
#		if fraction != 1.0: # this means moveit couldn't plan to the end
#			return 'moved_not_to_position'

		# When excute return value
		
		rospy.loginfo("box_bei_move_x: %s",round(get_coordinates.x, 2))
		rospy.loginfo("box_bei_move_y: %s",round(get_coordinates.y, 2))
		rospy.loginfo("box_bei_move_z: %s",get_coordinates.z)
		# Set the target of the group and then move the group to the specified target
#		mgc.go(target_pose)
		rospy.sleep(5.0)
		# Return a motion plan (a RobotTrajectory) to the set goal state
#		plan = mgc.plan(target_pose)
		# Execute a previously planned path
		mgc.execute(plan)
		return 'succeeded'
###############################################################################

### sub state machines
class pick_in_bin_move(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['moved_to_position', 'moved_not_to_position', 'error'])		
	
		with self:
			smach.StateMachine.add('GET_BOX_COORDINATES', get_coordinates(),
				transitions={'succeeded':'MOVE_TO_BOX', 
							'failed':'moved_not_to_position',
							'error':'error'})

			smach.StateMachine.add('MOVE_TO_BOX', move_to_box(),
				transitions={'succeeded':'MOVE_EE_ORIENTATION',
							'failed':'moved_not_to_position',
							'error':'error'})

			smach.StateMachine.add('MOVE_EE_ORIENTATION', move_ee_orientation(),
				transitions={'succeeded':'SHOOT_PIC_FROM_BOX',
							'failed':'moved_not_to_position',
							'error':'error'})

			smach.StateMachine.add('SHOOT_PIC_FROM_BOX', shoot_pic(),
				transitions={'succeeded':'moved_to_position', 
							'failed':'moved_not_to_position',
							'error':'error'})
