#!/usr/bin/env python
#-*-coding: utf8-*-

"""
Remplace le systeme VICON pour la simulation en recuperant la position des objets
dans gazebo et en les publiant.
Met egalement a jour les positions des objectifs des drones dans gazebo (invisible
en realite).
"""

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped

pos_leader = [0,0,0,0]
pos_follower = [0,0,0,0]
ms_leader = ModelState()
ms_follower = ModelState()

def recupere_pos(msg):
	global pos_leader, pos_follower

    #leader
	i = msg.name.index("uav1")
	pos_leader[0] = msg.pose[i].position.x
	pos_leader[1] = msg.pose[i].position.y
	pos_leader[2] = msg.pose[i].position.z
	q = (msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w)
	euler = euler_from_quaternion(q)
	pos_leader[3] = euler[2]

    #follower
	i = msg.name.index("uav2")
	pos_follower[0] = msg.pose[i].position.x
	pos_follower[1] = msg.pose[i].position.y
	pos_follower[2] = msg.pose[i].position.z
	q = (msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w)
	euler = euler_from_quaternion(q)
	pos_follower[3] = euler[2]

def recupere_cible_leader(msg):
    global ms_leader
    ms_leader.pose = msg.pose

def recupere_cible_follower(msg):
    global ms_follower
    ms_follower.pose = msg.pose


rospy.init_node('gazebo_vicon')

sub_pos = rospy.Subscriber('/gazebo/model_states', ModelStates, recupere_pos)
pub_pos_leader = rospy.Publisher('/vicon/ardrone_leader/ardrone_leader', TransformStamped, queue_size=1)
pub_pos_follower = rospy.Publisher('/vicon/ardrone_follower/ardrone_follower', TransformStamped, queue_size=1)

pos_l = TransformStamped()
pos_l.header.frame_id = "world"
pos_f = TransformStamped()
pos_f.header.frame_id = "world"

sub_obj_leader = rospy.Subscriber('/uav1/cible', PoseStamped, recupere_cible_leader)
sub_obj_follower = rospy.Subscriber('/uav2/cible', PoseStamped, recupere_cible_follower)
pub_obj_leader = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
pub_obj_follower = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

ms_leader.model_name = "obj_leader"
ms_leader.reference_frame = "world"

ms_follower.model_name = "objectif"
ms_follower.reference_frame = "world"

rate = rospy.Rate(20)

while not rospy.is_shutdown():
    pos_l.header.stamp = rospy.Time()
    q = quaternion_from_euler(0, 0, pos_leader[3])
    pos_l.transform.translation.x = pos_leader[0]
    pos_l.transform.translation.y = pos_leader[1]
    pos_l.transform.translation.z = pos_leader[2]
    pos_l.transform.rotation.x = q[0]
    pos_l.transform.rotation.y = q[1]
    pos_l.transform.rotation.z = q[2]
    pos_l.transform.rotation.w = q[3]

    pos_f.header.stamp = rospy.Time()
    q = quaternion_from_euler(0, 0, pos_follower[3])
    pos_f.transform.translation.x = pos_follower[0]
    pos_f.transform.translation.y = pos_follower[1]
    pos_f.transform.translation.z = pos_follower[2]
    pos_f.transform.rotation.x = q[0]
    pos_f.transform.rotation.y = q[1]
    pos_f.transform.rotation.z = q[2]
    pos_f.transform.rotation.w = q[3]

    pub_pos_leader.publish(pos_l)
    pub_pos_follower.publish(pos_f)

    pub_obj_leader.publish(ms_leader)
    pub_obj_follower.publish(ms_follower)

    rate.sleep()
