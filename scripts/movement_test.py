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
    # Open the container
    with sq:
        # Sequence.add('MOVEHOME', SimpleActionState('/Roboy/MoveEndEffector',
        #                                            roboy_communication_control.msg.MoveEndEffectorAction,
        #                                            goal=roboy_communication_control.msg.MoveEndEffectorGoal(
        #                                                endEffector='wrist_left_1',
        #                                                type=2,
        #                                                q_target=[0,0,0,0,0,-0.2,0], sendToRealHardware=False)))
        Sequence.add('MOVEHOME', SimpleActionState('/Roboy/MoveEndEffector',
                                                   roboy_communication_control.msg.MoveEndEffectorAction,
                                                   goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                       endEffector='wrist_left_1',
                                                       type=2,
                                                       q_target=[0, 0, 0, 0, 0, 0, 0], sendToRealHardware=True,
                                                       timeout=30, tolerance=0.1)))
        Sequence.add('MOVEUP', SimpleActionState('/Roboy/MoveEndEffector',
                                                 roboy_communication_control.msg.MoveEndEffectorAction,
                                                 goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                     endEffector='wrist_left_1',
                                                     type=2,
                                                     q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
                                                     timeout=30, tolerance=0.1)))
        Sequence.add('MOVEELBOW0', SimpleActionState('/Roboy/MoveEndEffector',
                                                     roboy_communication_control.msg.MoveEndEffectorAction,
                                                     goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                         endEffector='wrist_left_1',
                                                         type=2,
                                                         q_target=[0, -1.5, 0, 0, 0.5, 0, 0], sendToRealHardware=True,
                                                         timeout=30, tolerance=0.1)))
        Sequence.add('MOVEELBOW1', SimpleActionState('/Roboy/MoveEndEffector',
                                                     roboy_communication_control.msg.MoveEndEffectorAction,
                                                     goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                         endEffector='wrist_left_1',
                                                         type=2,
                                                         q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
                                                         timeout=30, tolerance=0.1)))
        Sequence.add('MOVEFRONT', SimpleActionState('/Roboy/MoveEndEffector',
                                                    roboy_communication_control.msg.MoveEndEffectorAction,
                                                    goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                        endEffector='wrist_left_1',
                                                        type=2,
                                                        q_target=[0, -1.5, -1.1, 0, 0, 0, 0], sendToRealHardware=True,
                                                        timeout=30, tolerance=0.1)))
        Sequence.add('MOVEFRONT2', SimpleActionState('/Roboy/MoveEndEffector',
                                                     roboy_communication_control.msg.MoveEndEffectorAction,
                                                     goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                         endEffector='wrist_left_1',
                                                         type=2,
                                                         q_target=[0, -1.5, -0.5, 0, 0, 0, 0],
                                                         sendToRealHardware=True,
                                                         timeout=30, tolerance=0.1)))
        Sequence.add('MOVEUP2', SimpleActionState('/Roboy/MoveEndEffector',
                                                  roboy_communication_control.msg.MoveEndEffectorAction,
                                                  goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                      endEffector='wrist_left_1',
                                                      type=2,
                                                      q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
                                                      timeout=30, tolerance=0.1)))
        Sequence.add('MOVEHOME2', SimpleActionState('/Roboy/MoveEndEffector',
                                                    roboy_communication_control.msg.MoveEndEffectorAction,
                                                    goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                                                        endEffector='wrist_left_1',
                                                        type=2,
                                                        q_target=[0, 0, 0, 0, 0, 0, 0], sendToRealHardware=True,
                                                        timeout=30, tolerance=0.1)))

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
