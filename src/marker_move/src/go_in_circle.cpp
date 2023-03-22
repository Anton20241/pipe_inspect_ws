#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
geometry_msgs::Twist velCommand2Aruco; // скорость для маркера [м/с]
ros::Publisher velocityPub;
double velocity_coeff = 0;
double radius_tube = 0; //(м) - радиус кривизны оси трубы

void setVelocity() {
  velCommand2Aruco.linear.x = 0.0;
  velCommand2Aruco.linear.y = velocity_coeff;
  velCommand2Aruco.linear.z = 0.0;
  velCommand2Aruco.angular.x = 0.0;
  velCommand2Aruco.angular.y = 0.0;
  velCommand2Aruco.angular.z = (-1.0) * velocity_coeff / radius_tube;

  velocityPub.publish(velCommand2Aruco);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_in_circle");
  ros::NodeHandle node;
  ros::param::param<double>("~Velocity_coeff", velocity_coeff, 0);
  ros::param::param<double>("~Radius_tube", radius_tube, 0);
  velocityPub =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  while (ros::ok()) {
    setVelocity();
  }
  return 0;
}