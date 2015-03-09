#!/usr/bin/env python

import roslib
import rospy

import smach
import smach_ros
from std_msgs.msg import String

# import scenario specific states and (sub-)state machines
from ur5_pick_in_bin_states import *

class pick_in_bin_trash(smach.StateMachine):
	def __init__(self, source_area):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

		with self:
			smach.StateMachine.add('PICK_OBJECT', move_lin(source_area),
				transitions={'object_picked':'PLACE_OBJECT', 
							'object_not_picked':'failed',	
							'error':'error'})

#			smach.StateMachine.add('PLACE_OBJECT', place_object(target_area),
	#			transitions={'object_placed':'succeeded',
	#						'object_not_placed':'failed',
		#					'error':'error'})

# main
def main():
	rospy.init_node('state_machine')
	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed', 'error'])

	# open the container
	with SM:
		smach.StateMachine.add('MOVE_TO_INIT', move_to_init(),
			transitions={	'succeeded':'PICK_IN_BIN_MOVE',
							'failed':'overall_failed'})

		smach.StateMachine.add('PICK_IN_BIN_MOVE', pick_in_bin_move(),
			transitions={	'moved_to_position':'MOVE_TO_INIT',
							'moved_not_to_position':'overall_failed',
							'error':'error'})
	
	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('PICK_IN_BIN', SM, 'PICK_IN_BIN')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

if __name__ == '__main__':
	main()
