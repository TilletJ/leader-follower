#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

/*
  Ce code permet le contrôle d'un Crazyflie avec une seule boule VICON. On lit
  donc sa position sur le topic /vicon/pos et cela ne permet pas d'avoir plusieurs
  drone simultanément dans la volière.
  Pour utiliser le Crazyflie avec d'autres drones, utiliser plutôt le noeud
  cfdrone_3B (3B signifiant ici 3 boules, le nombre minimum nécessaire).
*/


const double pi = 3.14159;
double cible[3], pos[3];
double roll, pitch, yaw;
double obj_roll, obj_pitch, obj_yaw;

void recuperePos(const geometry_msgs::Vector3::ConstPtr& msg)
{
  pos[0] = msg->x;
  pos[1] = msg->y;
  pos[2] = msg->z;
}

void recupereAngles(const geometry_msgs::Vector3::ConstPtr& msg)
{
  roll  = msg->x;
  pitch = msg->y;
  yaw   = msg->z;
}

void recuperePosCible(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cible[0] = msg->pose.position.x;
  cible[1] = msg->pose.position.y;
  cible[2] = msg->pose.position.z;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  obj_yaw = tf::getYaw(q);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "cfdrone");
  ros::NodeHandle n;

  std::string ns = ros::this_node::getNamespace();
  // On publie la commande au drone directement sur le topic /cmd_vel
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // On a besoin de récupérer la position du drone, la cible du drone, et les
  // angles du drone (qui ne sont pas donnés par le VICON comme il n'y a qu'une boule).
  ros::Subscriber sub_pos = n.subscribe("/vicon/pos", 1000, recuperePos);
  ros::Subscriber sub_cible = n.subscribe("/uav1/cible", 1000, recuperePosCible);
  ros::Subscriber sub_angles = n.subscribe("/angles", 1000, recupereAngles);

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
    e_cap = 0.0-yaw;

    uc = 40 * e_y + 20 * (e_y-e_y_old) / dt + 2 * m_integral_y;
    uc = std::max(std::min(10.0, uc), -10.0);
    vc = -40 * e_x - 20 * (e_x-e_x_old) / dt - 2 * m_integral_x;
    vc = std::max(std::min(10.0, vc), -10.0);

    // On projette la commande par rapport à l'orientation du drone.
    cmd.linear.x = cos(yaw*pi/180)*uc + sin(yaw*pi/180)*vc;
    cmd.linear.y = sin(yaw*pi/180)*uc - cos(yaw*pi/180)*vc;
    
    cmd.linear.z = 15000 + 5000 * e_z + 6000 * (e_z-e_z_old) / dt + 3500 * m_integral_z;
    cmd.linear.z = std::max(std::min(60000.0, cmd.linear.z), 10000.0);
    cmd.angular.z = -200 * e_cap - 20 * (e_cap-e_cap_old) / dt;
    cmd.angular.z = std::max(std::min(200.0, cmd.angular.z), -200.0);

    m_previousTime = time;
    pub_cmd.publish(cmd); // On publie la commande.
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
