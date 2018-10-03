#!/usr/bin/env python
import roslib

roslib.load_manifest('smach')
import rospy

import threading

import smach
from smach import StateMachine, Concurrence, State, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
import actionlib
import random
import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg
iterations = 1000
iteration = 0
sendtohardware = True
rospy.init_node('msj_platform')
moveEndeffector = actionlib.SimpleActionClient('/Roboy/MoveEndEffector', roboy_communication_control.msg.MoveEndEffectorAction)
lookat = actionlib.SimpleActionClient('/Roboy/LookAt', roboy_communication_control.msg.LookAtAction)

# class LookAt(State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['found', 'not found'], input_keys=['target'])
#         lookat.wait_for_server()
#     def execute(self, userdata):
#         global lookat
#         global sendtohardware
#         rospy.loginfo('Looking for cup')
#         goal=roboy_communication_control.msg.LookAtGoal(
#             endEffector='head',
#             root_frame='head',
#             target_frame=userdata.target,
#             yaw_joint_index=2,
#             pitch_joint_index=3,
#             type=2,
#             sendToRealHardware=sendtohardware,
#             timeout=10, tolerance=0.1)
#         # goal = roboy_communication_control.msg.MoveEndEffectorGoal(
#         #     endEffector='head',
#         #     type=2,
#         #     q_target=userdata, sendToRealHardware=sendtohardware,
#         #     timeout=30, tolerance=0.01)
#         lookat.send_goal(goal)
#         success = lookat.wait_for_result()
#         if success:
#             return 'found'
#         else:
#             return 'not found'

class randomPose(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','not done'])
    def execute(self, userdata):
        global iteration
        global iterations
        global moveEndeffector
        global sendtohardware
        goal=roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='top',
            type=2,
            q_target=[random.uniform(-0.6, 0.6),random.uniform(-0.6, 0.6),random.uniform(-0.8, 0.8)],
            sendToRealHardware=sendtohardware,
            timeout=10, tolerance=0.01)
        moveEndeffector.send_goal(goal)
        iteration = iteration+1
        moveEndeffector.wait_for_result()
        if iteration<iterations:
            return 'not done'
        else:
            goal=roboy_communication_control.msg.MoveEndEffectorGoal(
                endEffector='top',
                type=2,
                q_target=[0,0,0],
                sendToRealHardware=sendtohardware,
                timeout=20, tolerance=0.01)
            moveEndeffector.send_goal(goal)
            moveEndeffector.wait_for_result()
            return 'done'

def main():
    global lookat
    global moveEndeffector
    rospy.loginfo("waiting for movement servers to become available")
    lookat.wait_for_server()
    moveEndeffector.wait_for_server()

    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,0,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0.5,0,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[-0.5,0,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,0,0.5],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,0,-0.5],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,0.5,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,-0.5,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()
    # goal=roboy_communication_control.msg.MoveEndEffectorGoal(
    #     endEffector='top',
    #     type=2,
    #     q_target=[0,0,0],
    #     sendToRealHardware=sendtohardware,
    #     timeout=10, tolerance=0.01)
    # moveEndeffector.send_goal(goal)
    # moveEndeffector.wait_for_result()


    rospy.loginfo("starting state machine now")
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['success'])
    sm_top.userdata.cup = 0

    with sm_top:
        smach.StateMachine.add('RANDOMPOSE', randomPose(),
                           transitions={'done':'success', 'not done':'RANDOMPOSE'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Attach a SMACH introspection server
    sis = IntrospectionServer('msj_platform', sm_top, '/MSJ_PLATFORM_TEST')
    sis.start()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target=sm_top.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
