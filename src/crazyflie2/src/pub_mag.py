#!/usr/bin/env python
#-*-coding: utf8-*-

import rospy
from math import pi
from sensor_msgs.msg import MagneticField, Imu
from geometry_msgs.msg import Vector3, Vector3Stamped
from tf.transformations import euler_from_quaternion


def recupereMag(msg):
    magn = Vector3Stamped()
    magn.header = msg.header
    magn.vector = msg.magnetic_field
    pub_magn.publish(magn)

def recupereImu(msg):
    msg.orientation.w = 1
    pub_imu.publish(msg)

def recupereAngles(msg):
    q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = euler_from_quaternion(q)
    vec = Vector3()
    vec.x = euler[0]*180/pi
    vec.y = euler[1]*180/pi
    vec.z = euler[2]*180/pi
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    pub_angles.publish(vec)

rospy.init_node('pub_mag')

sub_magn = rospy.Subscriber('/magnetic_field', MagneticField, recupereMag)
pub_magn = rospy.Publisher('/imu/mag', Vector3Stamped, queue_size=1)
sub_imu = rospy.Subscriber('/imu', Imu, recupereImu)
pub_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)

sub_imu_orientation = rospy.Subscriber('/imu/data', Imu, recupereAngles)
pub_angles = rospy.Publisher('/angles', Vector3, queue_size=1)

rospy.spin()
