#include <string.h>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include "hector_uav_msgs/EnableMotors.h"

float cible[4], pos[4];
const double pi = 3.14159;
std::string ns;

void recuperePosL(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  if (ns[5] == 49) {
    pos[0] = msg->transform.translation.x;
    pos[1] = msg->transform.translation.y;
    pos[2] = msg->transform.translation.z;
    tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
    pos[3] = tf::getYaw(q);
  }
}

void recuperePosF(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  if (ns[5] == 50) {
    pos[0] = msg->transform.translation.x;
    pos[1] = msg->transform.translation.y;
    pos[2] = msg->transform.translation.z;
    tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
    pos[3] = tf::getYaw(q);
  }
}

void recuperePosCible(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cible[0] = msg->pose.position.x;
  cible[1] = msg->pose.position.y;
  cible[2] = msg->pose.position.z;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  cible[3] = tf::getYaw(q);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "drone");
  ros::NodeHandle n;

  ns = ros::this_node::getNamespace();
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 1000);
  ros::Subscriber sub_cible = n.subscribe(ns + "/cible", 1000, recuperePosCible);

  ros::Subscriber sub_pos_leader = n.subscribe("/vicon/ardrone_leader/ardrone_leader", 1000, recuperePosL);
  ros::Subscriber sub_pos_follower = n.subscribe("/vicon/ardrone_follower/ardrone_follower", 1000, recuperePosF);

  ros::ServiceClient client = n.serviceClient<hector_uav_msgs::EnableMotors>(ns + "/enable_motors");
  hector_uav_msgs::EnableMotors armement;
  armement.request.enable = true;
  if (client.call(armement))
  {
    ROS_INFO("Armement: %d", (bool)armement.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service enable_motors");
    return 1;
  }

  ros::Rate loop_rate(25);
  float cmd_max = 2.0;
  geometry_msgs::Twist cmd;
  float e_x, e_y, e_z, e_cap;
  float cmdx, cmdy;

  while (ros::ok()) {
    e_x = cible[0] - pos[0];
    e_y = cible[1] - pos[1];
    e_z = cible[2] - pos[2];
    e_cap = cible[3] - pos[3];
    if (e_cap > pi)
    {
      e_cap = e_cap - 2*pi;
    } else if (e_cap < -pi)
    {
      e_cap = e_cap + 2*pi;
    }

    if (e_x > 0.1)
    {
      cmd.linear.x = std::min(e_x, cmd_max);
    } else if (e_x < -0.1)
    {
      cmd.linear.x = std::max(e_x, -cmd_max);
    }

    if (e_y > 0.1)
    {
      cmd.linear.y = std::min(e_y, cmd_max);
    } else if (e_y < -0.1)
    {
      cmd.linear.y = std::max(e_y, -cmd_max);
    }

    if (e_z > 0.1)
    {
      cmd.linear.z = std::min(e_z, cmd_max);
    } else if (e_z < -0.1)
    {
      cmd.linear.z = std::max(e_z, -cmd_max);
    }

    if (e_cap > 0.0)
    {
      cmd.angular.z = 10 * std::min(e_cap, 100*cmd_max);
    } else //if (e_cap < -0.0)
    {
      cmd.angular.z = 10 * std::max(e_cap, -100*cmd_max);
    }

    cmdx = cmd.linear.x;
    cmdy = cmd.linear.y;
    cmd.linear.x = cos(pos[3])*cmdx + sin(pos[3])*cmdy;
    cmd.linear.y = -sin(pos[3])*cmdx + cos(pos[3])*cmdy;

    pub_cmd.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
