#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Empty.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

/*
  Ce code permet le contrôle d'un AR Drone follower. Il doit avoir son modèle sur
  le système VICON au nom de ardrone_follower.
*/

const double pi = 3.14159;
double cible[3], pos[3];
double yaw, obj_yaw;

ros::Publisher pub_landing;

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
}

void recupereEtatLeader(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
  if (msg->state == 2 || msg->state == 8 || msg->state == 0)
  {
    std_msgs::Empty empty;
    pub_landing.publish(empty);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ardrone_follower");
  ros::NodeHandle n;

  // On publie la commande au drone sur le topic /uav2/goal_vel pour que le PID prenne en compte l'objectif
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/uav2/goal_vel", 1000);
  // On a besoin de récupérer la position du drone, et la cible du drone.
  ros::Subscriber sub_pos = n.subscribe("/vicon/ardrone_follower/ardrone_follower", 1000, recuperePos);
  ros::Subscriber sub_cible = n.subscribe("/uav2/cible", 1000, recuperePosCible); // ns + "/cible"
  // On récupère également les informations de vol du leader pour savoir quand est-ce qu'il atterit.
  ros::Subscriber sub_navdata_leader = n.subscribe("/uav1/ardrone/navdata", 1000, recupereEtatLeader);
  // Si le leader atterit, on publie alors un message pour faire également atterir le follower.
  pub_landing = n.advertise<std_msgs::Empty>("/uav2/ardrone/land", 1000);

  ros::Rate loop_rate(50); // Les commandes sont envoyées à une fréquence de 50Hz.
  geometry_msgs::Twist goal;

  double e_cap;

  while (ros::ok()) {
    /*
      Ici, on utilise un régulateur PID (implémenter dans le package falkor_ardrone)
      pour contrôler le drone Parrot. On a juste besoin de diminuer la sortie de
      ce régulateur. Les sorties du PID ne sont pas multipliées par exactement
      les même valeurs que pour le leader, car lors des essais de ce projet,
      le leader était un AR Drone 1.0 tandis que le follower était la version 2.0.
      Les 2 drones ne réagissaient pas tout à fait pareil.
    */

    e_cap = obj_yaw - yaw;
    if (e_cap > pi)
    {
      e_cap = e_cap - 2*pi;
    } else if (e_cap < -pi)
    {
      e_cap = e_cap + 2*pi;
    }

    // On projette la commande par rapport à l'orientation du drone.
    goal.linear.x = 0.18 * (cos(yaw)*(cible[0]-pos[0]) + sin(yaw)*(cible[1]-pos[1]));
    goal.linear.y = 0.18 * (-sin(yaw)*(cible[0]-pos[0]) + cos(yaw)*(cible[1] - pos[1]));
    goal.linear.z = 0.4 * (cible[2] - pos[2]);
    goal.angular.z = 0.2 * e_cap;

    pub_cmd.publish(goal); // On publie la commande.
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
