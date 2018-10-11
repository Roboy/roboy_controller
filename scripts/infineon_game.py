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
import std_srvs.srv, geometry_msgs.msg, std_msgs.msg
import roboy_communication_control.msg
from pyquaternion import Quaternion
import numpy
p_cup_0 = geometry_msgs.msg.Point()
p_cup_1 = geometry_msgs.msg.Point()
p_cup_2 = geometry_msgs.msg.Point()
pos_table = numpy.array([0, 0, 0])
rot_table = Quaternion()
GOGOGO = False
use_aruco = False
sendtohardware = False

rospy.init_node('roboy_infineon_game')
listener = tf.TransformListener()
moveEndeffectorLeft = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                                   roboy_communication_control.msg.MoveEndEffectorAction)
moveEndeffectorRight = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_right_1',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)
lookat = actionlib.SimpleActionClient('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction)

smach_pub = rospy.Publisher('roboy/control/smach', std_msgs.msg.String, queue_size=1)

def gogogo(req):
    rospy.loginfo("gogogo")
    global GOGOGO
    if not GOGOGO:
        GOGOGO = True

    return std_srvs.srv.TriggerResponse(GOGOGO,'')

class getReady(State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'not ready'])

    def execute(self, userdata):
        global moveEndeffectorLeft
        global moveEndeffectorRight
        global sendtohardware
        global rot_table
        global pos_table
        global GOGOGO
        global smach_pub
        global use_aruco

        msg = std_msgs.msg.String()

        if GOGOGO:
            msg.data = "GETTINGREADY"
            smach_pub.publish(msg)
            goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                endEffector='wrist_left_1',
                type=2,
                q_target=[0, -1.5, 0, 0, 1.5, 0, 0], sendToRealHardware=sendtohardware,
                timeout=10, tolerance=0.1)
            moveEndeffectorLeft.send_goal(goal)
            # goal = roboy_communication_control.msg.MoveEndEffectorGoal(
            #     endEffector='wrist_right_1',
            #     type=2,
            #     q_target=[0, 0.2, 0, 0, 1, 0, 0], sendToRealHardware=sendtohardware,
            #     timeout=10, tolerance=0.1)
            # moveEndeffectorRight.send_goal(goal)
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
            if moveEndeffectorLeft.wait_for_result() and lookat.wait_for_result():
                userdata.cup = 0
                if use_aruco:
                    try:
                        (trans,rot) = listener.lookupTransform('/world', '/zed_left_aruco_69', rospy.Time(0))
                        pos_table = numpy.array([trans[0],trans[1],trans[2]])
                        rot_table = Quaternion([rot[0], rot[1], rot[2], rot[3]])
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.loginfo("could not find aruco marker")
                        return 'not ready'
                else:
                    pos_table = numpy.array([0.5,-0.5,0.5])
                    rot_table = Quaternion([0,0,0,1])
                msg.data = "READY"
                smach_pub.publish(msg)
                GOGOGO = False
                return 'ready'
            else:
                return 'not ready'
        else:
            rospy.loginfo_throttle(1,"waiting for go signal from dialog")
            msg.data = "WAITINGFORGO"
            smach_pub.publish(msg)
            return 'not ready'


class moveToCup(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'not done'], input_keys=['cup'], output_keys=['cup'])

    def execute(self, userdata):
        global moveEndeffectorLeft
        global moveEndeffectorRight
        global sendtohardware
        global pos_table
        global rot_table
        global smach_pub
        goal=roboy_communication_control.msg.LookAtGoal(
            endEffector='head',
            root_frame='head',
            target_frame='wrist_left_1',
            yaw_joint_index=2,
            pitch_joint_index=3,
            type=1,
            sendToRealHardware=sendtohardware,
            timeout=5, tolerance=0.1)
        lookat.send_goal(goal)
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = 0.4864
        pose.orientation.y = 0.54329
        pose.orientation.z = 0.47472
        pose.orientation.w = -0.49285
        msg = std_msgs.msg.String()
        if userdata.cup == 0:
            rospy.loginfo('Moving to cup 0')
            pos = numpy.array([0.05, -0.05, 0.])
            pos = rot_table.rotate(pos)
            pos = pos + pos_table
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            msg.data = "MOVINGTOCUP0"
            smach_pub.publish(msg)
        elif userdata.cup == 1:
            rospy.loginfo('Moving to cup 1')
            pos = numpy.array([0.05, -0.05-0.133, 0.])
            pos = rot_table.rotate(pos)
            pos = pos + pos_table
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            msg.data = "MOVINGTOCUP1"
            smach_pub.publish(msg)
        elif userdata.cup == 2:
            rospy.loginfo('Moving to cup 2')
            pos = numpy.array([0.05, -0.05-0.133*2, 0.])
            pos = rot_table.rotate(pos)
            pos = pos + pos_table
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            msg.data = "MOVINGTOCUP2"
            smach_pub.publish(msg)
        else:
            msg.data = "DONE"
            smach_pub.publish(msg)
            return 'done'
        rospy.loginfo("cup %d position %f %f %f"%(userdata.cup,pose.position.x,pose.position.y,pose.position.z))

        userdata.cup = userdata.cup + 1
        goal = roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='wrist_left_1',
            type=0,
            pose=pose,
            sendToRealHardware=sendtohardware,
            timeout=5, tolerance=0.01)
        moveEndeffectorLeft.send_goal(goal)
        goal = roboy_communication_control.msg.LookAtGoal(
            endEffector='head',
            root_frame='head',
            target_frame='wrist_left_1',
            yaw_joint_index=2,
            pitch_joint_index=3,
            type=2,
            sendToRealHardware=sendtohardware,
            timeout=20, tolerance=0.001)
        lookat.send_goal(goal)
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
    rospy.loginfo("waiting for head to become available")
    lookat.wait_for_server()
    rospy.loginfo("waiting for shoulder_left to become available")
    moveEndeffectorLeft.wait_for_server()
    rospy.loginfo("waiting for shoulder_right to become available")
    moveEndeffectorRight.wait_for_server()
    s = rospy.Service('roboy/cupgame/start', std_srvs.srv.Trigger, gogogo)
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
