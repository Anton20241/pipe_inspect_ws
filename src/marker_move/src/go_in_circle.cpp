#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>

using namespace std;

geometry_msgs::Twist velCmd2Aruco;        // скорость для маркера [м/с]
geometry_msgs::Pose currentArucoOdomPose; // текущее положение маркера

bool getCurrentArucoOdomPoseCallback = false;

double velocity = 0;               // [м/c] - скорость для маркера
double radius_tube = 0;            // [м]   - радиус кривизны оси трубы

void setVelocity() {
  velCmd2Aruco.linear.x = 0.0;
  velCmd2Aruco.linear.y = velocity;
  velCmd2Aruco.linear.z = 0.0;
  velCmd2Aruco.angular.x = 0.0;
  velCmd2Aruco.angular.y = 0.0;
  velCmd2Aruco.angular.z = (-1.0) * velocity / radius_tube;   // [rad/s] - omega
}

void getCurrentArucoOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  currentArucoOdomPose.position.x = arucoGazeboMsg.pose[1].position.x;
  currentArucoOdomPose.position.y = arucoGazeboMsg.pose[1].position.y;
  currentArucoOdomPose.position.z = arucoGazeboMsg.pose[1].position.z;

  currentArucoOdomPose.orientation.w = arucoGazeboMsg.pose[1].orientation.w;
  currentArucoOdomPose.orientation.x = arucoGazeboMsg.pose[1].orientation.x;
  currentArucoOdomPose.orientation.y = arucoGazeboMsg.pose[1].orientation.y;
  currentArucoOdomPose.orientation.z = arucoGazeboMsg.pose[1].orientation.z;

  getCurrentArucoOdomPoseCallback = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_in_circle");
  ros::NodeHandle node;

  ros::param::param<double>("~Velocity", velocity, 0);
  ros::param::param<double>("~Radius_tube", radius_tube, 0);

  ros::Subscriber currentArucoOdomPoseSub =
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);
 
  ros::Publisher velCmd2ArucoPub =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

  ros::Rate loop_rate(100); 
    
  while (ros::ok()) {
    
    ros::spinOnce();
    if (!getCurrentArucoOdomPoseCallback) continue;
    getCurrentArucoOdomPoseCallback = false;
    setVelocity();
    velCmd2ArucoPub.publish(velCmd2Aruco);
    ROS_INFO("\n"
              "Current x = %.5f\n"
              "Current y = %.5f\n"
              "Current z = %.5f\n",
              currentArucoOdomPose.position.x, currentArucoOdomPose.position.y, currentArucoOdomPose.position.z);
    loop_rate.sleep();
  }

  return 0;
}