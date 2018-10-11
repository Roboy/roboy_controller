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

rospy.init_node('roboy_home')
lookat = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/head',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)

def main():
    global lookat
    rospy.loginfo("waiting for head to become available")
    lookat.wait_for_server()
    global lookat
    rospy.loginfo('Looking home')
    goal=roboy_communication_control.msg.MoveEndEffectorGoal(
        endEffector='head',
        type=2,
        q_target=[0, 0, 0, 0],
        sendToRealHardware=True,
        timeout=5, tolerance=0.0)
    lookat.send_goal(goal)
    lookat.wait_for_result()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
