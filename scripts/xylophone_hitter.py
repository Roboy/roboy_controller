#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

import threading

import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg

from tf import TransformListener

import numpy as np
import pyquaternion


class TF_Listener():
    #get one key position and move to key
    #TODO get all key positions
    def __init__(self, *args):
        self.tf = TransformListener()

    position = None
    def get_key_pos(self, key):
        #get key position
        while not (self.tf.frameExists("/world") and self.tf.frameExists(key)):
            import pdb
            # pdb.set_trace()
            #t = self.tf.getLatestCommonTime("/world", key)
            try:
                position = self.tf.lookupTransform(key, "/world", rospy.Time(0))
                break
            except:
                continue

        return position


def rotation_matrix(alpha,beta,gamma):
    #Rotation matrices
    Rx = np.matrix(([1,0,0],[0,np.cos(alpha),-np.sin(alpha)],[0,np.sin(alpha),np.cos(alpha)]))
    Ry = np.matrix(([np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]))
    Rz = np.matrix(([np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]))

    rotation_matrix = Rx.dot(Ry).dot(Rz)
    return rotation_matrix

def hit_key(key_pos):

    client = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                          roboy_communication_control.msg.MoveEndEffectorAction)
    client.wait_for_server()

    pose = geometry_msgs.msg.Pose()
    pose.position.x = key_pos[0][0]
    pose.position.y = key_pos[0][1]
    pose.position.z = key_pos[0][2]

    #rotate quaternion around
    quat = key_pos[1]
    #rot_quat = pyquaternion.Quaternion(axis=[-1, 0, -1], angle=(np.pi/2.))
    #TODO add rotation of quat
    #new_quat =

    pose.orientation.x = key_pos[1][0]
    pose.orientation.y = key_pos[1][1]
    pose.orientation.z = key_pos[1][2]
    pose.orientation.w = key_pos[1][3]

    print(pose.position.x, pose.position.y, pose.position.z)

    goal=roboy_communication_control.msg.MoveEndEffectorGoal(
        endEffector='wrist_left_1',
        type=0,
        pose=pose,
        sendToRealHardware=False,
        timeout=10,
        tolerance=0.1)

    print("send goal")
    client.send_goal(goal)
    print("sent goal")
    client.wait_for_result()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('xylophone_hitter')

    listener = TF_Listener()
    e_0 = listener.get_key_pos('/E_0')
    print("this", e_0)
    hit_key(e_0)

