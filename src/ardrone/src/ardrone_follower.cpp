#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

const double pi = 3.14159;
double cible[3], pos[3];
double yaw, obj_yaw;

void recuperePos(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  pos[0] = msg->transform.translation.x;
  pos[1] = msg->transform.translation.y;
  pos[2] = msg->transform.translation.z;
  tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  yaw = tf::getYaw(q);
}

void recuperePosCible(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cible[0] = msg->pose.position.x;
  cible[1] = msg->pose.position.y;
  cible[2] = msg->pose.position.z;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  obj_yaw = tf::getYaw(q);
  //TODO recuper le cap de la cible et s'en servir.
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ardrone_follower");
  ros::NodeHandle n;

  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/uav2/goal_vel", 1000);
  ros::Subscriber sub_pos = n.subscribe("/vicon/ardrone_follower/ardrone_follower", 1000, recuperePos);
  ros::Subscriber sub_cible = n.subscribe("/uav2/cible", 1000, recuperePosCible); // ns + "/cible"

  ros::Rate loop_rate(50);
  geometry_msgs::Twist goal;

  double e_cap;

  while (ros::ok()) {

    e_cap = obj_yaw - yaw;
    if (e_cap > pi)
    {
      e_cap = e_cap - 2*pi;
    } else if (e_cap < -pi)
    {
      e_cap = e_cap + 2*pi;
    }

    goal.linear.x = 0.15 * (cos(yaw)*(cible[0]-pos[0]) + sin(yaw)*(cible[1]-pos[1]));
    goal.linear.y = 0.15 * (-sin(yaw)*(cible[0]-pos[0]) + cos(yaw)*(cible[1] - pos[1]));
    goal.linear.z = 0.15 * (cible[2] - pos[2]);
    goal.angular.z = 0.15 * e_cap;

    pub_cmd.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
