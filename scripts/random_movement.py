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
import roboy_communication_control.srv
sendtohardware = True

emotions = ['shy','smileblink','angry','tongue_out','kiss','pissed','happy','happy2','hypno','hypno_color','rolling','surprised']
shoulder_left_movements = ['shoulder_left_movement0', 'shoulder_left_movement1']
shoulder_right_movements = ['shoulder_right_movement0', 'shoulder_right_movement1', 'shoulder_right_movement2', 'shoulder_right_movement3']

rospy.init_node('roboy_random_movement')
shoulder_left = actionlib.SimpleActionClient('/shoulder_left_movements_server',
                                               roboy_communication_control.msg.PerformMovementsAction)
shoulder_right = actionlib.SimpleActionClient('/shoulder_right_movements_server',
                                             roboy_communication_control.msg.PerformMovementsAction)
emotion_srv = rospy.ServiceProxy('roboy/cognition/face/emotion', roboy_communication_control.srv.ShowEmotion)

class RandomMovement(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        global emotions
        global emotion_srv
        global shoulder_left
        global shoulder_right
        global shoulder_left_movements
        global shoulder_right_movements
        global sendtohardware

        emotion_srv(random.choice(emotions))
        rospy.loginfo('random movement shoulder left')
        goal = roboy_communication_control.msg.PerformMovementsGoal(
            actions=[random.choice(shoulder_left_movements)])
        shoulder_left.send_goal(goal)
        rospy.sleep(random.uniform(0, 5))

        rospy.loginfo('random movement shoulder right')
        goal = roboy_communication_control.msg.PerformMovementsGoal(
            actions=[random.choice(shoulder_right_movements)])
        shoulder_right.send_goal(goal)
        rospy.sleep(random.uniform(0, 5))

        for i in range(0,int(random.uniform(4, 7))):
            emotion_srv(random.choice(emotions))
            rospy.sleep(random.uniform(4, 6))

        if shoulder_left.wait_for_result() and shoulder_right.wait_for_result():
            return 'done'

def main():
    global shoulder_left
    global shoulder_right
    rospy.loginfo("waiting for shoulders to become available")
    shoulder_left.wait_for_server()
    shoulder_right.wait_for_server()
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.cup = 0

    # Open the container
    with sm_top:
        smach.StateMachine.add('RANDOMMOVEMENT', RandomMovement(),
                               transitions={'done': 'RANDOMMOVEMENT'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Attach a SMACH introspection server
    sis = IntrospectionServer('roboy_random_movement', sm_top, '/RANDOMMOVEMENT')
    sis.start()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target=sm_top.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
