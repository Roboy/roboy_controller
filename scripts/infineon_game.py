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
import roboy_communication_cognition.srv
from pyquaternion import Quaternion
import numpy
from numpy.linalg import inv
p_cup_0 = geometry_msgs.msg.Point()
p_cup_1 = geometry_msgs.msg.Point()
p_cup_2 = geometry_msgs.msg.Point()
pos_table = numpy.array([0, 0, 0])
rot_table = Quaternion()
GOGOGO = False
use_aruco = False
sendtohardware = False
userecorded = True
trigger_joint_controller = False
ball_detected = 0
use_head = False

rospy.init_node('roboy_infineon_game')
listener = tf.TransformListener()
moveEndeffectorLeft = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                                   roboy_communication_control.msg.MoveEndEffectorAction)
enable_elbow_joint = rospy.ServiceProxy('roboy/middleware/elbow_left/JointController', std_srvs.srv.SetBool)
enable_wrist_joint = rospy.ServiceProxy('roboy/middleware/wrist_left/JointController', std_srvs.srv.SetBool)
speech = rospy.ServiceProxy('roboy/cognition/speech/synthesis/talk', roboy_communication_cognition.srv.Talk)
moveEndeffectorRight = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_right_1',
                                                    roboy_communication_control.msg.MoveEndEffectorAction)
lookat = actionlib.SimpleActionClient('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction)
moveEndeffectorHead = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/head',
                                      roboy_communication_control.msg.MoveEndEffectorAction)

performMovement = actionlib.SimpleActionClient('/shoulder_left_movements_server',
                                                   roboy_communication_control.msg.PerformMovementsAction)

smach_pub = rospy.Publisher('roboy/control/smach', std_msgs.msg.String, queue_size=1)
ball_messages = 0
cup_ball = [0,0]
ball_detector = ()

def ball_cb(data):
    global i
    global ball_detected
    global ball_detector
    global ball_messages
    ball_messages = ball_messages+1
    ball_detected = ball_detected + int(data.data)
    if ball_messages>=300:
        ball_detected = ball_detected
        ball_detector.unregister()
        rospy.loginfo(ball_detected)


def gogogo(data):
    rospy.loginfo("gogogo")
    global GOGOGO
    if not GOGOGO:
        GOGOGO = True

class getReady(State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'not ready'])

    def execute(self, userdata):
        global moveEndeffectorLeft
        global moveEndeffectorRight
        global moveEndeffectorHead
        global performMovement
        global sendtohardware
        global rot_table
        global pos_table
        global GOGOGO
        global smach_pub
        global use_aruco
        global use_head

        msg = std_msgs.msg.String()

        if GOGOGO:
            msg.data = "GETTINGREADY"
            smach_pub.publish(msg)
            if not userecorded:
                goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                    endEffector='wrist_left_1',
                    type=2,
                    q_target=[0, -1.5, 0, 0, 1.5, 0, 0], sendToRealHardware=sendtohardware,
                    timeout=3, tolerance=0.1)
                moveEndeffectorLeft.send_goal(goal)
                goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                    endEffector='wrist_right_1',
                    type=2,
                    q_target=[0, 0.2, 0, 0, 1, 0, 0], sendToRealHardware=sendtohardware,
                    timeout=3, tolerance=0.1)
                moveEndeffectorRight.send_goal(goal)
                if use_head:
                    goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                        endEffector='head',
                        type=2,
                        q_target=[0, 0, 0.5, 0], sendToRealHardware=sendtohardware,
                        timeout=3, tolerance=0.1)
                    moveEndeffectorHead.send_goal(goal)
                if moveEndeffectorLeft.wait_for_result():
                    userdata.cup = 0
                    if use_aruco:
                        try:
                            (trans,rot) = listener.lookupTransform('/world', '/zed_left_aruco_69', rospy.Time(0))
                            rospy.loginfo(rot)
                            pos_table = numpy.array([trans[0],trans[1],trans[2]])
                            rot_table = Quaternion([rot[0], rot[1], rot[2], rot[3]])
                            rot_table = rot_table.normalised
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.loginfo("could not find aruco marker")
                            return 'not ready'
                    else:
                        pos_table = numpy.array([0.3,-0.6,0.5])
                        rot_table = Quaternion([0,0,0,1])
                    msg.data = "READY"
                    smach_pub.publish(msg)
                    GOGOGO = False
                    return 'ready'
                else:
                    return 'not ready'
            else:
                GOGOGO = False
                goal = roboy_communication_control.msg.PerformMovementsGoal(
                    actions=['shoulder_left_getready'])
                performMovement.send_goal(goal)
                if performMovement.wait_for_result():
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
        global enable_elbow_joint
        global enable_wrist_joint
        global trigger_joint_controller
        global ball_detector
        global ball_messages
        global use_head
        global ball_detected
        pose = geometry_msgs.msg.Pose()
        # pose.orientation.x = 0.4864
        # pose.orientation.y = 0.54329
        # pose.orientation.z = 0.47472
        # pose.orientation.w = -0.49285

        msg = std_msgs.msg.String()
        if userdata.cup == 0:
            rospy.loginfo('Moving to cup 0')
            msg.data = "MOVINGTOCUP0"
            smach_pub.publish(msg)
            if userecorded:
                goal = roboy_communication_control.msg.PerformMovementsGoal(
                    actions=['shoulder_left_cup0'])
                performMovement.send_goal(goal)
                performMovement.wait_for_result()
                msg.data = "MEASURINGCUP0"
                smach_pub.publish(msg)
                ball_messages = 0
                ball_detected = 0
                ball_detector = rospy.Subscriber('roboy/soli/ball', std_msgs.msg.Bool, ball_cb)
                rospy.loginfo('waiting for ball detector')
                if ball_messages>=100 and ball_detected:
                    msg.data = "DONE"
                    smach_pub.publish(msg)
                    rospy.loginfo('the ball is in cup 0')
                    return 'done'

            else:
                pos = numpy.array([0.8,-0.2,0.5])
                # pos = rot_table.rotate(pos)
                # rospy.loginfo(rot_table)
                # pos = pos + pos_table
                pose.orientation.x = -0.11474
                pose.orientation.y = -0.67661
                pose.orientation.z = 0.23839
                pose.orientation.w = 0.68708
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]

        elif userdata.cup == 1:
            rospy.loginfo('Moving to cup 1')
            msg.data = "MOVINGTOCUP1"
            smach_pub.publish(msg)
            if userecorded:
                goal = roboy_communication_control.msg.PerformMovementsGoal(
                    actions=['shoulder_left_cup1'])
                performMovement.send_goal(goal)
                performMovement.wait_for_result()
                msg.data = "MEASURINGCUP1"
                smach_pub.publish(msg)
                ball_messages = 0
                ball_detected = 0
                ball_detector = rospy.Subscriber('roboy/soli/ball', std_msgs.msg.Bool, ball_cb)
                rospy.loginfo('waiting for ball detector')
                if ball_messages>=100 and ball_detected:
                    msg.data = "DONE"
                    smach_pub.publish(msg)
                    rospy.loginfo('the ball is in cup 1')
                    return 'done'
            else:
                pos = numpy.array([0.8,0,0.5])
                # pos = rot_table.rotate(pos)
                # rospy.loginfo(rot_table)
                # pos = pos + pos_table
                pose.orientation.x = 0
                pose.orientation.y = -0.67337
                pose.orientation.z = 0
                pose.orientation.w = 0.73725
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]

        elif userdata.cup == 2:
            rospy.loginfo('Moving to cup 2')
            msg.data = "MOVINGTOCUP2"
            smach_pub.publish(msg)
            if userecorded:
                goal = roboy_communication_control.msg.PerformMovementsGoal(
                    actions=['shoulder_left_gohome'])
                performMovement.send_goal(goal)
                performMovement.wait_for_result()
                msg.data = "DONE"
                smach_pub.publish(msg)
                ball_messages = 0
                ball_detector = rospy.Subscriber('roboy/soli/ball', std_msgs.msg.Bool, ball_cb)
                rospy.loginfo('waiting for ball detector')
                if ball_messages>=100 and ball_detected:
                    msg.data = "DONE"
                    smach_pub.publish(msg)
                    rospy.loginfo('the ball is in cup 2')
                    return 'done'
            else:
                pos = numpy.array([0.8,0.2,0.5])
                # pos = rot_table.rotate(pos)
                # rospy.loginfo(rot_table)
                # pos = pos + pos_table
                pose.orientation.x = 0.24
                pose.orientation.y = -0.62
                pose.orientation.z = 0.08
                pose.orientation.w = 0.72
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
        else:
            if use_head:
                goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                    endEffector='head',
                    type=2,
                    q_target=[0, 0, 0, 0], sendToRealHardware=sendtohardware,
                    timeout=10, tolerance=0.1)
                moveEndeffectorHead.send_goal(goal)
            if userecorded:
                goal = roboy_communication_control.msg.PerformMovementsGoal(
                    actions=['shoulder_left_gohome'])
                performMovement.send_goal(goal)
                performMovement.wait_for_result()
            else:
                if sendtohardware:
                    if trigger_joint_controller:
                        enable_elbow_joint(True)
                        enable_wrist_joint(False)
                    goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                        endEffector='wrist_left_1',
                        type=2,
                        q_target=[0, -1.5, 0, 0, 0, 0, 0], sendToRealHardware=sendtohardware,
                        timeout=10, tolerance=0.1)
                    moveEndeffectorLeft.send_goal(goal)
                    moveEndeffectorLeft.wait_for_result()
                    goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                        endEffector='wrist_left_1',
                        type=2,
                        q_target=[0, 0, 0, 0, 0, 0, 0], sendToRealHardware=sendtohardware,
                        timeout=10, tolerance=0.1)
                    moveEndeffectorLeft.send_goal(goal)
                    moveEndeffectorLeft.wait_for_result()
                if sendtohardware:
                    if trigger_joint_controller:
                        enable_elbow_joint(False)
                        enable_wrist_joint(False)
            msg.data = "DONE"
            smach_pub.publish(msg)
            return 'done'
        rospy.loginfo("cup %d position %f %f %f"%(userdata.cup,pose.position.x,pose.position.y,pose.position.z))
        userdata.cup = userdata.cup + 1

        if userdata.cup <= 2:
            if not userecorded:
                if sendtohardware:
                    if trigger_joint_controller:
                        enable_elbow_joint(True)
                        enable_wrist_joint(True)
                goal = roboy_communication_control.msg.MoveEndEffectorGoal(
                    endEffector='wrist_left_1',
                    type=0,
                    pose=pose,
                    sendToRealHardware=sendtohardware,
                    timeout=30, tolerance=0.1)
                moveEndeffectorLeft.send_goal(goal)
                moveEndeffectorLeft.wait_for_result()
                if sendtohardware:
                    if trigger_joint_controller:
                        enable_elbow_joint(False)
                        enable_wrist_joint(False)
            return 'not done'


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

    req = roboy_communication_cognition.srv.TalkRequest()
    req.text = "hehehehhehe"
    speech(req)

    gogogogogo = rospy.Subscriber('roboy/control/smach/start', std_msgs.msg.Bool, gogogo)
    # s = rospy.Service('roboy/cupgame/start', std_srvs.srv.Trigger, gogogo)
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.cup = 0
    sm_top.userdata.cup_with_ball = 0

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
