#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>

using namespace std;

geometry_msgs::Twist velCmd2Aruco;            // скорость для маркера [м/с]
double velocity     = 0;                      // [м/c] - скорость для маркера
double radius_tube  = 0;                      // [м]   - радиус кривизны оси трубы

void setVelocity() {
  velCmd2Aruco.linear.x = 0.0;
  velCmd2Aruco.linear.y = velocity;
  velCmd2Aruco.linear.z = 0.0;
  velCmd2Aruco.angular.x = 0.0;
  velCmd2Aruco.angular.y = 0.0;
  velCmd2Aruco.angular.z = (-1.0) * velocity / radius_tube;   // [rad/s] - omega
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_in_circle");
  ros::NodeHandle node;

  ros::param::param<double>("~Velocity", velocity, 0);
  ros::param::param<double>("~Radius_tube", radius_tube, 0);
 
  ros::Publisher velCmd2ArucoPub =
    node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel", 0);
    
  setVelocity();
  velCmd2ArucoPub.publish(velCmd2Aruco);
  ros::spin();

  return 0;
}