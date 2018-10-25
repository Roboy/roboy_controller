#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

#import threading

import std_srvs.srv, geometry_msgs.msg
import roboy_communication_control.msg

from tf import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import pyquaternion

class Xylophone():
    def __init__(self):
        self.notes_list = ['C_0','C_sharp_0', 'D_0', 'D_sharp_0', 'E_0', 'F_0', 'F_sharp_0', 'G_0', 'G_sharp_0', 'A_0', 'A_sharp_0', 'H_0',
                            'C_1','C_sharp_1', 'D_1', 'D_sharp_1', 'E_1', 'F_1', 'F_sharp_1', 'G_1', 'G_sharp_1', 'A_1', 'A_sharp_1', 'H_1',
                            'C_2','C_sharp_2', 'D_2', 'D_sharp_2', 'E_2', 'F_2', 'F_sharp_2', 'G_2', 'G_sharp_2', 'A_2', 'A_sharp_2', 'H_2']

    def hit_key(self, key_pos):
        #action to make roboy hit key
        client = actionlib.SimpleActionClient('/Roboy/MoveEndEffector/wrist_left_1',
                                              roboy_communication_control.msg.MoveEndEffectorAction)
        client.wait_for_server()

        #move arm above xylophone
        client.send_goal(roboy_communication_control.msg.MoveEndEffectorGoal(
                         endEffector='wrist_left_1',
                         type=2,
                         q_target=[0, -1.5, -1.5, 0, 0, 0, 0], 
                         sendToRealHardware=False,
                         timeout=10, tolerance=0.1))
        client.wait_for_result()

        pose = geometry_msgs.msg.Pose()

        #not rotated
        # pose.position.x = -key_pos[0][0]
        # pose.position.y = -key_pos[0][1]
        # pose.position.z = -key_pos[0][2]
        # pose.orientation.x = key_pos[1][0]
        # pose.orientation.y = key_pos[1][1]
        # pose.orientation.z = key_pos[1][2]
        # pose.orientation.w = key_pos[1][3]
   
        #transform key_pos
        rot_quat = pyquaternion.Quaternion(axis=[-1, 0, -1], angle=(np.pi/2.))
        print('rot_quat', rot_quat)
        print('rot_quat[0]' ,rot_quat[0])
        new_key_pos = rot_quat.rotate(key_pos[0])
        #rotated
        pose.position.x = new_key_pos[0]
        pose.position.y = new_key_pos[1]
        pose.position.z = new_key_pos[2]
        pose.orientation.x = rot_quat[0]
        pose.orientation.y = rot_quat[1]
        pose.orientation.z = rot_quat[2]
        pose.orientation.w = rot_quat[3]

        #hardcoded
        # pose.position.x = -new_key_pos[0]
        # pose.position.y = -new_key_pos[1]
        # pose.position.z = -new_key_pos[2]
        # pose.orientation.x = -0.498746
        # pose.orientation.y = -0.498746
        # pose.orientation.z = -0.464615
        # pose.orientation.w = 0.53585

        print('key_pos', key_pos[0][0], key_pos[0][1], key_pos[0][2])
        print('rotated key_pos', pose.position.x, pose.position.y, pose.position.z)

        show_marker(pose)

        print("send goal")
        client.send_goal(roboy_communication_control.msg.MoveEndEffectorGoal(
            endEffector='wrist_left_1',
            type=0,
            pose=pose,
            sendToRealHardware=False,
            timeout=10,
            tolerance=0.1))
        print("sent goal")
        client.wait_for_result()

        # Signal handler
        rospy.spin()

class TF_Listener():
    #get one key position and move to key
    #TODO get all key positions
    def __init__(self, *args):
        self.tf = TransformListener()

    position = None
    def get_key_pos(self, key):
        #get key position
        while not (self.tf.frameExists("/world") and self.tf.frameExists(key)):
            #t = self.tf.getLatestCommonTime("/world", key)
            try:
                position = self.tf.lookupTransform(key, "/world", rospy.Time(0))
                break
            except:
                continue

        return position


def show_marker(pose, rate=1000, color=(1,0,0,1), sphereSize=0.1):

    topic = 'visualization_marker'
    publisher = rospy.Publisher(topic, Marker, queue_size=1000)
    rate3 = rospy.Rate(rate)

    markers = Marker()
    markers.header.frame_id = 'world'
    markers.ns = 'ns'
    markers.type = Marker.SPHERE
    markers.pose = pose
    markers.color.r = color[0]
    markers.color.g = color[1]
    markers.color.b = color[2]
    markers.color.a = color[3]

    markers.scale.x = sphereSize
    markers.scale.y = sphereSize
    markers.scale.z = sphereSize
    markers.lifetime = rospy.Duration(10);
    markers.header.stamp = rospy.Time.now()
    markers.action = Marker.ADD
    markers.id = 1001241


    publisher.publish(markers);
    print("marker published")
    rate3.sleep()

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('xylophone_hitter')

    xyl = Xylophone()
    listener = TF_Listener()

    for note in xyl.notes_list[:1]:
        print(note)
        current_key_pos = listener.get_key_pos(note)
        print("this", current_key_pos)
        xyl.hit_key(current_key_pos)