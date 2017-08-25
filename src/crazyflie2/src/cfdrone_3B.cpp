#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

/*
  Ce code permet le contrôle d'un Crazyflie avec au moins 3 boules VICON (d'où le '3B'). On lit
  sa position sur le topic /vicon/cf_leader/cf_leader.
*/


const double pi = 3.14159;
double cible[4], pos[4];

void recuperePos(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  pos[0] = msg->transform.translation.x;
  pos[1] = msg->transform.translation.y;
  pos[2] = msg->transform.translation.z;
  tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
  pos[4] = tf::getYaw(q);
}

void recuperePosCible(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cible[0] = msg->pose.position.x;
  cible[1] = msg->pose.position.y;
  cible[2] = msg->pose.position.z;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  cible[4] = tf::getYaw(q);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "cfdrone");
  ros::NodeHandle n;

  std::string ns = ros::this_node::getNamespace();
  // On publie la commande au drone directement sur le topic /cmd_vel
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // On récupère la position du drone et celle de la cible.
  ros::Subscriber sub_pos = n.subscribe("/vicon/cf_leader/cf_leader", 1000, recuperePos);
  ros::Subscriber sub_cible = n.subscribe("/uav1/cible", 1000, recuperePosCible);

  double e_x = 0, e_y = 0, e_z = 0, e_cap = 0, e_x_old, e_y_old, e_z_old, e_cap_old;
  ros::Time m_previousTime = ros::Time::now();
  double m_integral_x = 0, m_integral_y = 0, m_integral_z = 0;
  double uc, vc;
  float dt;
  ros::Time time;

  ros::Rate loop_rate(50); // Les commandes sont envoyées à une fréquence de 50Hz.
  geometry_msgs::Twist cmd;

  while (ros::ok()) {
    /*
      Ici, on utilise un régulateur PID pour contrôler le Crazyflie.
    */

    time = ros::Time::now();
    dt = time.toSec() - m_previousTime.toSec();
    if (dt <= 0) // dt ne doit pas être nul (pour éviter erreur div / 0)
        {
            dt = 0.02;
        }

    e_x_old = e_x;
    e_y_old = e_y;
    e_z_old = e_z;
    e_cap_old = e_cap;

    m_integral_x += e_x * dt;
    m_integral_y += e_y * dt;
    m_integral_z += e_z * dt;
    m_integral_x = std::max(std::min(m_integral_x, 0.1), -0.1);
    m_integral_y = std::max(std::min(m_integral_y, 0.1), -0.1);
    m_integral_z = std::max(std::min(m_integral_z, 1000.0), -1000.0);

    e_x = cible[0] - pos[0];
    e_y = cible[1] - pos[1];
    e_z = cible[2] - pos[2];
    e_cap = cible[4] - pos[4];

    uc = 40 * e_y + 20 * (e_y-e_y_old) / dt + 2 * m_integral_y;
    uc = std::max(std::min(10.0, uc), -10.0);
    vc = -40 * e_x - 20 * (e_x-e_x_old) / dt - 2 * m_integral_x;
    vc = std::max(std::min(10.0, vc), -10.0);

    // On projette la commande par rapport à l'orientation du drone.
    cmd.linear.x = cos(pos[4])*uc + sin(pos[4])*vc;
    cmd.linear.y = sin(pos[4])*uc - cos(pos[4])*vc;

    cmd.linear.z = 35000 + 5000 * e_z + 6000 * (e_z-e_z_old) / dt + 3500 * m_integral_z;
    cmd.linear.z = std::max(std::min(60000.0, cmd.linear.z), 30000.0);
    cmd.angular.z = -200 * e_cap - 20 * (e_cap-e_cap_old) / dt;
    cmd.angular.z = std::max(std::min(40.0, cmd.angular.z), -40.0);

    m_previousTime = time;
    pub_cmd.publish(cmd); // On publie la commande.
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
