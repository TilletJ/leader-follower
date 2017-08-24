#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>

const double pi = 3.14159;
float X[3];
float t = 0;

void euler(const float& dt)
{
  float x_old = X[0];
  float y_old = X[1];

  //Lissajoux
  //X[0] = 15*sin(t/9.)*dt;
  //X[1] = 15*sin(2*t/9.+pi/8.0)*dt;

  //Cercle
  //X[0] = 15*sin(t/9.)*dt;
  //X[1] = 15*cos(t/9.)*dt;
  //X[2] = atan2(X[1] - y_old, X[0] - x_old);

  // Lemniscate (symbole infini)
  X[0] = 1.5 * sqrt(2) * sin(t/25.) / (1 + pow(cos(t/25.),2));
  X[1] = 3.0 * sqrt(2) * sin(t/25.) * cos(t/25.) / (1 + pow(cos(t/25.),2));
  X[2] = atan2(X[1] - y_old, X[0] - x_old);

  //X[0] = 15*t/9.*dt;
  //X[1] = std::min(15*t/9.*dt - 2, 2.0);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "commande_leader");
  ros::NodeHandle n;

  ros::Publisher cible = n.advertise<geometry_msgs::PoseStamped>("/uav1/cible", 1000);

  X[0]=0;
  X[1]=0;
  X[2]=0;

  ros::Rate loop_rate(25);
  float dt = 0.1;
  tf::Quaternion q;
  geometry_msgs::PoseStamped msg;

  while (ros::ok()) {
    euler(dt);
    t += dt;

    q.setRPY(0,0,X[2]);

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = X[0];
    msg.pose.position.y = X[1];
    msg.pose.position.z = 1.0;
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
