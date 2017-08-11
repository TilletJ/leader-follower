#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/ModelState.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

const double pi = 3.14159;
double pos_leader[4] = {0}, v_vitesse[2];
double theta = 0, d = 1.3;

void recupere_pos(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  pos_leader[0] = msg->transform.translation.x;
	pos_leader[1] = msg->transform.translation.y;
	pos_leader[2] = msg->transform.translation.z;
  pos_leader[3] = tf::getYaw(msg->transform.rotation);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "commande_follower");
  ros::NodeHandle n;

  ros::Subscriber sub_pos = n.subscribe("/vicon/ardrone_leader/ardrone_leader", 1000, recupere_pos);
  ros::Publisher cible = n.advertise<geometry_msgs::PoseStamped>("/uav2/cible", 1000);

  ros::Rate loop_rate(25);

  geometry_msgs::PoseStamped msg;
  tf::Quaternion q;
  msg.header.frame_id = "world";

  while (ros::ok()) {
    theta = pos_leader[3] + pi;

    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = pos_leader[0] + d*cos(theta);
    msg.pose.position.y = pos_leader[1] + d*sin(theta);
    msg.pose.position.z = std::max(pos_leader[2] - 0.2, 1.0);
    q.setRPY(0,0,theta - pi);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();

    cible.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
