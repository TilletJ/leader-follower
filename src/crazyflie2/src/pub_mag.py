#!/usr/bin/env python
#-*-coding: utf8-*-

import rospy
from math import pi
from sensor_msgs.msg import MagneticField, Imu
from geometry_msgs.msg import Vector3, Vector3Stamped
from tf.transformations import euler_from_quaternion

"""
    Ce noeud sert à récupérer le cap du Crazyflie et à le publier. En effet, avec
    le firmware actuel, on n'a pas directement accès aux angles d'Euler.
    Ce programme utilise le package imu_tools qui donne les angles yaw pitch et roll
    à partir des données imu (accéléro, magnetic_field, ...)
    Pour que le package imu_tools fonctionne correctement, on est obligé de republier
    des topics déjà existant sous d'autres noms, avec de légères modifications.
"""

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
# Finalement, on publie les données qui nous intéressent dans le topic /angles.
pub_angles = rospy.Publisher('/angles', Vector3, queue_size=1)

rospy.spin()
