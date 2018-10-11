#!/usr/bin/env python
import roslib

roslib.load_manifest('smach')
import rospy

import threading
import random
import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import actionlib
import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg

rospy.init_node('roboy_look_around')
lookat = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/head',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)

class LookAt(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        lookat.wait_for_server()

    def execute(self, userdata):
        global lookat
        global sendtohardware
        rospy.loginfo('Looking around')
        goal=roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='head',
            type=2,
            q_target=[0, 0, random.uniform(-0.1, 0.5), random.uniform(-0.6, 0.6)],
            sendToRealHardware=True,
            timeout=5, tolerance=0.01)
        lookat.send_goal(goal)
        lookat.wait_for_result()
        return 'done'

def main():
    global lookat
    rospy.loginfo("waiting for head to become available")
    lookat.wait_for_server()
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.cup = 0

    # Open the container
    with sm_top:
        smach.StateMachine.add('LOOKAROUND', LookAt(),
                               transitions={'done': 'LOOKAROUND'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Attach a SMACH introspection server
    sis = IntrospectionServer('roboy_look_around', sm_top, '/LOOKAROUND')
    sis.start()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target=sm_top.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
