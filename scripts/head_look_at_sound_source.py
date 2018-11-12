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
import roboy_communication_cognition.msg
import numpy as np
import math

rospy.init_node('roboy_audio_location')
lookat = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/head',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)
location = np.array([0.0,0.0,0.0])
new_goal = False

def callback(data):
    global location
    global new_goal
    sound_sources = len(data.x)
    if sound_sources>0:
        random_source = random.randint(0,sound_sources-1)
        rospy.loginfo_throttle(1,"there are " + str(sound_sources) + " sound sources, choosing " + str(random_source))
        location[0] = data.x[random_source]
        location[1] = data.y[random_source]
        location[2] = data.z[random_source]

        # rospy.loginfo_throttle(1,str(location[0]) + ' ' + str(location[1]) + ' ' + str(location[2]))
        new_goal  = True

def main():
    global lookat
    rospy.loginfo("waiting for head to become available")
    lookat.wait_for_server()
    rospy.loginfo("head is ready")

    rospy.Subscriber("/roboy/cognition/audio/record/location", roboy_communication_cognition.msg.AudioLocation, callback)
    global new_goal
    global location
    while not rospy.is_shutdown():
        if new_goal:
            yaw = -0.7*location[0]
            pitch = 0
            if location[1]>0:
                pitch = 0.1+0.2*location[1]

            rospy.loginfo("pitch: " + str(pitch) + "\t\tyaw" + str(yaw))
            goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                endEffector='head',
                type=2,
                q_target=[0, 0, pitch, yaw],
                sendToRealHardware=True,
                timeout=5, tolerance=0.0)
            lookat.send_goal(goal)
            lookat.wait_for_result()
            new_goal = False

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
