#!/usr/bin/env python
import roslib

roslib.load_manifest('smach')
import rospy

import threading

import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer

import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg


def main():
    rospy.init_node('roboy_movement_test')

    # Create a SMACH state machine
    sq = Sequence(
        outcomes=['succeeded', 'aborted', 'preempted'],
        connector_outcome='succeeded')
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0
    pose.position.y = 0.06
    pose.position.z = 0.1
    pose.orientation.x = 0.4864
    pose.orientation.y = 0.54329
    pose.orientation.z = 0.47472
    pose.orientation.w = -0.49285
    point = geometry_msgs.msg.Vector3()
    point.x = 0
    point.y = -1
    point.z = 0.5
    # Open the container
    with sq:
        # Sequence.add('MOVEHOME', SimpleActionState('/Roboy/MoveEndEffector',
        #                                            roboy_communication_control.msg.MoveEndEffectorAction,
        #                                            goal=roboy_communication_control.msg.MoveEndEffectorGoal(
        #                                                endEffector='head',
        #                                                type=2,
        #                                                q_target=[0, 0, 0, 0], sendToRealHardware=True,
        #                                                timeout=30, tolerance=0.01)))
        Sequence.add('TRACKARUCO69', SimpleActionState('/Roboy/LookAt',
                                                   roboy_communication_control.msg.LookAtAction,
                                                   goal=roboy_communication_control.msg.LookAtGoal(
                                                       endEffector='head',
                                                       root_frame='head',
                                                       target_frame='zed_left_aruco_69',
                                                       yaw_joint_index=2,
                                                       pitch_joint_index=3,
                                                       point=point,
                                                       type=2,
                                                       sendToRealHardware=False,
                                                       timeout=10, tolerance=0.01)))
        # Sequence.add('MOVEHOME2', SimpleActionState('/Roboy/MoveEndEffector',
        #                                             roboy_communication_control.msg.MoveEndEffectorAction,
        #                                             goal=roboy_communication_control.msg.MoveEndEffectorGoal(
        #                                                 endEffector='head',
        #                                                 type=2,
        #                                                 q_target=[0, 0, 0, 0], sendToRealHardware=True,
        #                                                 timeout=30, tolerance=0.01)))

    # Attach a SMACH introspection server
    sis = IntrospectionServer('roboy_infineon_game', sq, '/INFINEON_GAME')
    sis.start()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target=sq.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
