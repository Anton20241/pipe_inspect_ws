#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

using namespace std;

geometry_msgs::Twist velCmd2Aruco; // скорость для маркера [м/с]
geometry_msgs::Pose currentArucoOdomPose; // текущее положение маркера
geometry_msgs::Pose desiredArucoOdomPose; // желаемое положение маркера

double goal_x = 0;
double goal_y = 0;
double goal_z = 0;

bool getCurrentArucoOdomPoseCallback = false;
 
double velocity = 0;
double move_precision = 0;

bool goal_x_reached = false;
bool goal_y_reached = false;
bool goal_z_reached = false;

void setup() {
 
  desiredArucoOdomPose.position.x = goal_x;
  desiredArucoOdomPose.position.y = goal_y;
  desiredArucoOdomPose.position.z = goal_z;

  velCmd2Aruco.linear.x = 0.0;
  velCmd2Aruco.linear.y = 0.0;
  velCmd2Aruco.linear.z = 0.0;
  velCmd2Aruco.angular.x = 0.0;
  velCmd2Aruco.angular.y = 0.0;
  velCmd2Aruco.angular.z = 0.0;
}

double getDistanceToGoal(const double desired, const double current) {
  return desired - current;
}
 
void setVelocity() {

  double getDistanceToGoal_x = getDistanceToGoal(desiredArucoOdomPose.position.x, currentArucoOdomPose.position.x);
  double getDistanceToGoal_y = getDistanceToGoal(desiredArucoOdomPose.position.y, currentArucoOdomPose.position.y);
  double getDistanceToGoal_z = getDistanceToGoal(desiredArucoOdomPose.position.z, currentArucoOdomPose.position.z);
  ROS_INFO("\n"
          "getDistanceToGoal_x = %.5f\n"
          "getDistanceToGoal_y = %.5f\n"
          "getDistanceToGoal_z = %.5f",
          getDistanceToGoal_x, getDistanceToGoal_y, getDistanceToGoal_z);

  //velocity x
  if (abs(getDistanceToGoal_x) > move_precision) {
    velCmd2Aruco.linear.x = velocity * getDistanceToGoal_x / abs(getDistanceToGoal_x);
  }
  else {
    ROS_INFO("goal_x_reached!");
    goal_x_reached = true;
    velCmd2Aruco.linear.x = 0;
  }

  //velocity y
  if (abs(getDistanceToGoal_y) > move_precision) {
    velCmd2Aruco.linear.y = velocity * getDistanceToGoal_y / abs(getDistanceToGoal_y);
  }
  else {
    ROS_INFO("goal_y_reached!");
    goal_y_reached = true;
    velCmd2Aruco.linear.y = 0;
  }

  //velocity z
  if (abs(getDistanceToGoal_z) > move_precision) {
    velCmd2Aruco.linear.z = velocity * getDistanceToGoal_z / abs(getDistanceToGoal_z);
  }
  else {
    ROS_INFO("goal_z_reached!");
    goal_z_reached = true;
    velCmd2Aruco.linear.z = 0;
  }
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

  ros::init(argc, argv, "go_to_goal_xyz");
  ros::NodeHandle node;

  ros::param::param<double>("~Goal_x", goal_x, 0);
  ros::param::param<double>("~Goal_y", goal_y, 0);
  ros::param::param<double>("~Goal_z", goal_z, 0);
  ros::param::param<double>("~Move_precision", move_precision, 0);
  ros::param::param<double>("~Velocity", velocity, 0);

  setup();
 
  ros::Subscriber currentArucoOdomPoseSub =
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);
 
  ros::Publisher velCmd2ArucoPub =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

  ros::Rate loop_rate(100); 
    
  while (ros::ok() && (!goal_x_reached || !goal_y_reached || !goal_z_reached)) {
    
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

  ROS_INFO("\n"
          "goal_x has been reached!\n"
          "goal_y has been reached!\n"
          "goal_z has been reached!\n");

  return 0;
}