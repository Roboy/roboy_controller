#!/usr/bin/env python
"""
Description:

Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_06.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata:
            []
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine transitioning 'SPAWN':'succeeded'-->'TELEPORT1'
    [INFO] : State machine transitioning 'TELEPORT1':'succeeded'-->'TELEPORT2'
    [INFO] : State machine transitioning 'TELEPORT2':'succeeded'-->'DRAW_SHAPES'
    [INFO] : Concurrence starting with userdata:
            []
    [WARN] : Still waiting for action server 'turtle_shape2' to start... is it running?
    [WARN] : Still waiting for action server 'turtle_shape1' to start... is it running?
    [INFO] : Connected to action server 'turtle_shape2'.
    [INFO] : Connected to action server 'turtle_shape1'.
    [INFO] : Concurrent Outcomes: {'SMALL': 'succeeded', 'BIG': 'succeeded'}
    [INFO] : State machine terminating 'DRAW_SHAPES':'succeeded':'succeeded'
"""
import roslib;

roslib.load_manifest('smach')
import rospy

import threading

import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer

import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg


def main():
    rospy.init_node('roboy_infineon_game')

    # Create a SMACH state machine
    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0
    pose.position.y = 0.06
    pose.position.z = 0.1
    pose.orientation.x = 0.4864
    pose.orientation.y = 0.54329
    pose.orientation.z = 0.47472
    pose.orientation.w = -0.49285
    # Open the container
    with sq:
        Sequence.add('LOOKDOWN',
                     SimpleActionState('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction,
                                       goal=roboy_communication_control.msg.LookAtGoal(
                                           endEffector='head',
                                           yaw_joint_index=2,
                                           pitch_joint_index=3,
                                           type=1,
                                           point=geometry_msgs.msg.Vector3(0,-2,0),
                                           root_frame='zed_camera_left_lense',
                                           target_frame='cup_aruco_1'
                                       )))

        Sequence.add('LOOKFORCUP0',
                         SimpleActionState('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction,
                                           goal=roboy_communication_control.msg.LookAtGoal(
                                               endEffector='head',
                                               yaw_joint_index=2,
                                               pitch_joint_index=3,
                                               type=1,
                                               point=geometry_msgs.msg.Vector3(),
                                               root_frame='zed_camera_left_lense',
                                               target_frame='cup_aruco_0'
                                           )))
        Sequence.add('MOVETOCUP0', SimpleActionState('/Roboy/MoveEndEffector',
                                                        roboy_communication_control.msg.MoveEndEffectorAction,
                                                        goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                            pose=pose,
                                                            endEffector='wrist_left_1',
                                                            type=1,
                                                            target_frame='cup_aruco_0')))
        Sequence.add('LOOKFORCUP1',
                     SimpleActionState('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction,
                                       goal=roboy_communication_control.msg.LookAtGoal(
                                           endEffector='head',
                                           yaw_joint_index=2,
                                           pitch_joint_index=3,
                                           type=1,
                                           point=geometry_msgs.msg.Vector3(),
                                           root_frame='zed_camera_left_lense',
                                           target_frame='cup_aruco_1'
                                       )))
        Sequence.add('MOVETOCUP1', SimpleActionState('/Roboy/MoveEndEffector',
                                                     roboy_communication_control.msg.MoveEndEffectorAction,
                                                     goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                         pose=pose,
                                                         endEffector='wrist_left_1',
                                                         type=1,
                                                         target_frame='cup_aruco_1')))
        Sequence.add('LOOKFORCUP2',
                     SimpleActionState('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction,
                                       goal=roboy_communication_control.msg.LookAtGoal(
                                           endEffector='head',
                                           yaw_joint_index=2,
                                           pitch_joint_index=3,
                                           type=1,
                                           point=geometry_msgs.msg.Vector3(),
                                           root_frame='zed_camera_left_lense',
                                           target_frame='cup_aruco_2'
                                       )))
        Sequence.add('MOVETOCUP2', SimpleActionState('/Roboy/MoveEndEffector',
                                                     roboy_communication_control.msg.MoveEndEffectorAction,
                                                     goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                         pose=pose,
                                                         endEffector='wrist_left_1',
                                                         type=1,
                                                         target_frame='cup_aruco_2')))

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
