#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

geometry_msgs::Twist velCommand2Aruco; // скорость для маркера [м/с]
geometry_msgs::PoseStamped currentArucoGazeboOdomPose; // текущее факт положение маркера
geometry_msgs::PoseStamped desiredArucoGazeboOdomPose; // желаемое факт положение маркера

double goal_x = 0;
double goal_y = 0;
double goal_z = 0;

bool getCurrentArucoGazeboOdomPoseCallback = false;
 
double velocity_coeff = 0;
double move_precision = 0;

bool goal_x_reached = false;
bool goal_y_reached = false;
bool goal_z_reached = false;

void setup() {
 
  desiredArucoGazeboOdomPose.pose.position.x = goal_x;
  desiredArucoGazeboOdomPose.pose.position.y = goal_y;
  desiredArucoGazeboOdomPose.pose.position.z = goal_z;

  velCommand2Aruco.linear.x = 0.0;
  velCommand2Aruco.linear.y = 0.0;
  velCommand2Aruco.linear.z = 0.0;
  velCommand2Aruco.angular.x = 0.0;
  velCommand2Aruco.angular.y = 0.0;
  velCommand2Aruco.angular.z = 0.0;
}

double getDistanceTogGoal(const double currentCoordinate, const double prevCoordinate) {
  return currentCoordinate - prevCoordinate;
}
 
void setVelocity() {

  double getDistanceTogGoal_x = getDistanceTogGoal(desiredArucoGazeboOdomPose.pose.position.x, currentArucoGazeboOdomPose.pose.position.x);
  double getDistanceTogGoal_y = getDistanceTogGoal(desiredArucoGazeboOdomPose.pose.position.y, currentArucoGazeboOdomPose.pose.position.y);
  double getDistanceTogGoal_z = getDistanceTogGoal(desiredArucoGazeboOdomPose.pose.position.z, currentArucoGazeboOdomPose.pose.position.z);

  ROS_INFO("\ngetDistanceTogGoal_x = %.5f\n"
          "getDistanceTogGoal_y = %.5f\n"
          "getDistanceTogGoal_z = %.5f\n",
          getDistanceTogGoal_x, getDistanceTogGoal_y, getDistanceTogGoal_z);

  //velocity x
  if (goal_x_reached) {;}
  else if (abs(getDistanceTogGoal_x) > move_precision) {
    velCommand2Aruco.linear.x = velocity_coeff * getDistanceTogGoal_x / abs(getDistanceTogGoal_x);
  }
  else {
    goal_x_reached = true;
    velCommand2Aruco.linear.x = 0;
  }

  //velocity y
  if (goal_y_reached) {;}
  else if (abs(getDistanceTogGoal_y) > move_precision) {
    velCommand2Aruco.linear.y = velocity_coeff * getDistanceTogGoal_y / abs(getDistanceTogGoal_y);
  }
  else {
    goal_y_reached = true;
    velCommand2Aruco.linear.y = 0;
  }

  //velocity z
  if (goal_z_reached) {;}
  else if (abs(getDistanceTogGoal_z) > move_precision) {
    velCommand2Aruco.linear.z = velocity_coeff * getDistanceTogGoal_z / abs(getDistanceTogGoal_z);
  }
  else {
    goal_z_reached = true;
    velCommand2Aruco.linear.z = 0;
  }
}

void getCurrentArucoGazeboOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  currentArucoGazeboOdomPose.pose.position.x = arucoGazeboMsg.pose[1].position.x;
  currentArucoGazeboOdomPose.pose.position.y = arucoGazeboMsg.pose[1].position.y;
  currentArucoGazeboOdomPose.pose.position.z = arucoGazeboMsg.pose[1].position.z;

  currentArucoGazeboOdomPose.pose.orientation.w = arucoGazeboMsg.pose[1].orientation.w;
  currentArucoGazeboOdomPose.pose.orientation.x = arucoGazeboMsg.pose[1].orientation.x;
  currentArucoGazeboOdomPose.pose.orientation.y = arucoGazeboMsg.pose[1].orientation.y;
  currentArucoGazeboOdomPose.pose.orientation.z = arucoGazeboMsg.pose[1].orientation.z;

  getCurrentArucoGazeboOdomPoseCallback = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "go_to_goal_xyz");
  ros::NodeHandle node;

  ros::param::param<double>("~Goal_x", goal_x, 0);
  ros::param::param<double>("~Goal_y", goal_y, 0);
  ros::param::param<double>("~Goal_z", goal_z, 0);
  ros::param::param<double>("~Move_precision", move_precision, 0);
  ros::param::param<double>("~Velocity_coeff", velocity_coeff, 0);

  setup();
 
  ros::Subscriber arucoGazeboPoseSub =
    node.subscribe("/gazebo/model_states", 100, getCurrentArucoGazeboOdomPose);
 
  ros::Publisher velocityPub =
    node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
  ros::Rate loop_rate(30);

  while (ros::ok() && ((!goal_x_reached) || (!goal_y_reached) || (!goal_z_reached))) {

    if (getCurrentArucoGazeboOdomPoseCallback){
      setVelocity();
      velocityPub.publish(velCommand2Aruco);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("\ngoal_x has been reached!\n"
          "goal_y has been reached!\n"
          "goal_z has been reached!\n");
          
  return 0;
}