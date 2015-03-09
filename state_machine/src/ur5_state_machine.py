#!/usr/bin/env python

import roslib
import rospy

import smach
import smach_ros
from std_msgs.msg import String

# import scenario specific states and (sub-)state machines
from ur5_pick_in_bin_states import *

# main
def main():
	rospy.init_node('state_machine')
	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed', 'error'])

	# open the container
	with SM:
		smach.StateMachine.add('MOVE_TO_INIT', move_to_init(),
			transitions={	'succeeded':'MOVE_TO_MAGAZIN',
							'failed':'overall_failed'})

		smach.StateMachine.add('MOVE_TO_MAGAZIN', move_to_magazin(),
			transitions={	'succeeded':'ROTATE_IN_PREGRASP',
							'failed':'overall_failed',
							'error':'error'})

#		smach.StateMachine.add('MOVE_TO_PREGRASP', move_to_pregrasp(),
#			transitions={	'succeeded':'ROTATE_IN_PREGRASP',
#							'failed':'overall_failed',
#							'error':'error'})

		smach.StateMachine.add('ROTATE_IN_PREGRASP', rotate_in_pregrasp(),
			transitions={	'succeeded':'overall_succeeded',
							'failed':'overall_failed',
							'error':'error'})

#		smach.StateMachine.add('MOVE_LIN', move_lin(),
#			transitions={	'succeeded':'overall_succeeded',
#							'failed':'overall_failed',
#							'error':'error'})

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
