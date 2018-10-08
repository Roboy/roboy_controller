#!/usr/bin/env python
import roslib

roslib.load_manifest('smach')
import rospy

import threading

import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import actionlib
import tf
import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg
p_cup_0 = geometry_msgs.msg.Point()
p_cup_1 = geometry_msgs.msg.Point()
p_cup_2 = geometry_msgs.msg.Point()
pose_table = geometry_msgs.msg.Pose()

sendtohardware = False
rospy.init_node('roboy_infineon_game')
listener = tf.TransformListener()
moveEndeffectorLeft = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                                   roboy_communication_control.msg.MoveEndEffectorAction)
moveEndeffectorRight = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)
lookat = actionlib.SimpleActionClient('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction)


class getReady(State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'not ready'])

    def execute(self, userdata):
        global moveEndeffectorLeft
        global moveEndeffectorRight
        global sendtohardware
        global pose_table
        rospy.loginfo('Getting Ready')
        goal = roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='wrist_left_1',
            type=2,
            q_target=[0, -1.5, 0, 0, 1.5, 0, 0], sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.1)
        moveEndeffectorLeft.send_goal(goal)
        goal = roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='wrist_right_1',
            type=2,
            q_target=[0, 0.2, 0, 0, 1, 0, 0], sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.1)
        moveEndeffectorRight.send_goal(goal)
        p = geometry_msgs.msg.Point()
        p.x = 0.3
        p.y = -1
        p.z = 0.1
        goal = roboy_communication_control.msg.LookAtGoal(
            endEffector='head',
            root_frame='head',
            yaw_joint_index=2,
            pitch_joint_index=3,
            point=p,
            type=0,
            sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.1)
        lookat.send_goal(goal)
        if moveEndeffectorLeft.wait_for_result() and moveEndeffectorRight.wait_for_result() and lookat.wait_for_result():
            userdata.cup = 0
            try:
                (trans,rot) = listener.lookupTransform('/world', '/zed_left_aruco_1', rospy.Time(0))
                pose_table.position = trans
                pose_table.orientation = rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("could not find aruco marker")
                return 'not ready'
            return 'ready'
        else:
            return 'not ready'


class moveToCup(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'not done'], input_keys=['cup'], output_keys=['cup'])

    def execute(self, userdata):
        global moveEndeffectorLeft
        global moveEndeffectorRight
        global sendtohardware
        global pose_table
        goal=roboy_communication_control.msg.LookAtGoal(
            endEffector='head',
            root_frame='head',
            target_frame='wrist_left_1',
            yaw_joint_index=2,
            pitch_joint_index=3,
            type=1,
            sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.2)
        lookat.send_goal(goal)
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = 0.4864
        pose.orientation.y = 0.54329
        pose.orientation.z = 0.47472
        pose.orientation.w = -0.49285
        if userdata.cup == 0:
            rospy.loginfo('Moving to cup 0')
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
        elif userdata.cup == 1:
            rospy.loginfo('Moving to cup 1')
            pose.position.x = 0.15
            pose.position.y = 0
            pose.position.z = 0
        elif userdata.cup == 2:
            rospy.loginfo('Moving to cup 2')
            pose.position.x = 0.3
            pose.position.y = 0
            pose.position.z = 0
        else:
            return 'done'
        pose.position.x = pose.position.x + pose_table.position[0]
        pose.position.y = pose.position.y + pose_table.position[1]
        pose.position.z = pose.position.z + pose_table.position[2]
        rospy.loginfo("cup %d position %f %f %f"%(userdata.cup,pose.position.x,pose.position.y,pose.position.z))

        userdata.cup = userdata.cup + 1
        goal = roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='wrist_left_1',
            type=0,
            pose=pose,
            sendToRealHardware=sendtohardware,
            timeout=20, tolerance=0.01)
        moveEndeffectorLeft.send_goal(goal)
        if userdata.cup <= 3:
            moveEndeffectorLeft.wait_for_result()
            return 'not done'
        else:
            return 'done'


class LookAt(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not found'], input_keys=['target'])
        lookat.wait_for_server()

    def execute(self, userdata):
        global lookat
        global sendtohardware
        rospy.loginfo('Looking for cup')
        goal = roboy_communication_control.msg.LookAtGoal(
            endEffector='head',
            root_frame='head',
            target_frame=userdata.target,
            yaw_joint_index=2,
            pitch_joint_index=3,
            type=2,
            sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.1)
        # goal = roboy_communication_control.msg.MoveEndEffectorGoal(
        #     endEffector='head',
        #     type=2,
        #     q_target=userdata, sendToRealHardware=sendtohardware,
        #     timeout=30, tolerance=0.01)
        lookat.send_goal(goal)
        success = lookat.wait_for_result()
        if success:
            return 'found'
        else:
            return 'not found'


def main():
    global lookat
    global moveEndeffector
    rospy.loginfo("waiting for endeffectors to become available")
    lookat.wait_for_server()
    moveEndeffectorLeft.wait_for_server()
    moveEndeffectorRight.wait_for_server()
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.cup = 0

    # Open the container
    with sm_top:
        smach.StateMachine.add('GETREADY', getReady(),
                               transitions={'ready': 'MEASURECUP', 'not ready': 'GETREADY'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['result'], input_keys=['cup'])

        # Open the container
        with sm_sub:
            smach.StateMachine.add('MOVETOCUP', moveToCup(),
                                   transitions={'done': 'result',
                                                'not done': 'MOVETOCUP'})
        smach.StateMachine.add('MEASURECUP', sm_sub,
                               transitions={'result': 'GETREADY'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Create a SMACH state machine
    # sq = Sequence(
    #     outcomes=['succeeded', 'aborted', 'preempted'],
    #     connector_outcome='succeeded')
    # pose = geometry_msgs.msg.Pose()
    # pose.position.x = 0
    # pose.position.y = 0.06
    # pose.position.z = 0.1
    # pose.orientation.x = 0.4864
    # pose.orientation.y = 0.54329
    # pose.orientation.z = 0.47472
    # pose.orientation.w = -0.49285
    # Open the container
    # with sq:
    # Sequence.add('MOVEHOME', SimpleActionState('/Roboy/MoveEndEffector',
    #                                            roboy_communication_control.msg.MoveEndEffectorAction,
    #                                            goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                endEffector='wrist_left_1',
    #                                                type=2,
    #                                                q_target=[0, 0, 0, 0, 0, 0, 0], sendToRealHardware=True,
    #                                                timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEUP', SimpleActionState('/Roboy/MoveEndEffector',
    #                                          roboy_communication_control.msg.MoveEndEffectorAction,
    #                                          goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                              endEffector='wrist_left_1',
    #                                              type=2,
    #                                              q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
    #                                              timeout=10, tolerance=0.1)))
    # Sequence.add('MOVEFRONT', SimpleActionState('/Roboy/MoveEndEffector',
    #                                          roboy_communication_control.msg.MoveEndEffectorAction,
    #                                          goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                              endEffector='wrist_left_1',
    #                                              type=2,
    #                                              q_target=[0, -1.5, -1, 0, 1.5, 0, 0], sendToRealHardware=True,
    #                                              timeout=10, tolerance=0.1)))
    # Sequence.add('MOVEELBOW0', SimpleActionState('/Roboy/MoveEndEffector',
    #                                              roboy_communication_control.msg.MoveEndEffectorAction,
    #                                              goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                  endEffector='wrist_left_1',
    #                                                  type=2,
    #                                                  q_target=[0, 0, 0, 0, 1.5, 0, 0], sendToRealHardware=True,
    #                                                  timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEELBOW1', SimpleActionState('/Roboy/MoveEndEffector',
    #                                              roboy_communication_control.msg.MoveEndEffectorAction,
    #                                              goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                  endEffector='wrist_left_1',
    #                                                  type=2,
    #                                                  q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
    #                                                  timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEFRONT', SimpleActionState('/Roboy/MoveEndEffector',
    #                                             roboy_communication_control.msg.MoveEndEffectorAction,
    #                                             goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                 endEffector='wrist_left_1',
    #                                                 type=2,
    #                                                 q_target=[0, -1.5, -1.1, 0, 0, 0, 0], sendToRealHardware=True,
    #                                                 timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEFRONT2', SimpleActionState('/Roboy/MoveEndEffector',
    #                                              roboy_communication_control.msg.MoveEndEffectorAction,
    #                                              goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                  endEffector='wrist_left_1',
    #                                                  type=2,
    #                                                  q_target=[0, -1.5, -0.5, 0, 0, 0, 0],
    #                                                  sendToRealHardware=True,
    #                                                  timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEUP2', SimpleActionState('/Roboy/MoveEndEffector',
    #                                           roboy_communication_control.msg.MoveEndEffectorAction,
    #                                           goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                               endEffector='wrist_left_1',
    #                                               type=2,
    #                                               q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=True,
    #                                               timeout=30, tolerance=0.1)))
    # Sequence.add('MOVEHOME2', SimpleActionState('/Roboy/MoveEndEffector',
    #                                             roboy_communication_control.msg.MoveEndEffectorAction,
    #                                             goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #                                                 endEffector='wrist_left_1',
    #                                                 type=2,
    #                                                 q_target=[0, 0, 0, 0, 0, 0, 0], sendToRealHardware=True,
    #                                                 timeout=30, tolerance=0.1)))

    # Attach a SMACH introspection server
    sis = IntrospectionServer('roboy_infineon_game', sm_top, '/INFINEON_GAME')
    sis.start()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target=sm_top.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
