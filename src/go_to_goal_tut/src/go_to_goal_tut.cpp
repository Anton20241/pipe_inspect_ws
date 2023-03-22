#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // x, y, theta, linear & angular velocity
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D desired; // Desired x, y, and theta 
 
// goal_x x-value, which can be any number from 0 to 11 (inclusive)
double goal_x = 0;
 
// The gain K, which is used to calculate the linear velocity
double k_l = 0;
 
// The distance threshold in meters that will determine when 
// the robot successfully reaches the goal_x.
double precision_x = 0;

bool goal_x_reached = false;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // Desired x goal_x coordinate
  desired.x = goal_x;
   
  // Initialize the Twist message.
  // Initial linear and angular velocities are 0 m/s and rad/s, respectively.
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Get the distance between the current x coordinate and 
// the desired x coordinate.
double getDistanceTogoal_x() {
  return desired.x - current.x;
}
 
// If we haven't yet reached the goal_x, set the velocity value.
// Otherwise, stop the robot.
void setVelocity() {
  if (abs(getDistanceTogoal_x()) > precision_x) {
 
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot is from the 
    // goal_x.
    velCommand.linear.x = k_l * getDistanceTogoal_x();
  }
  else {
    ROS_INFO("goal_x has been reached!\n\n");
    velCommand.linear.x = 0;
    goal_x_reached = true;
  }
}
 
// This callback function updates the current position and 
// orientation of the robot. 
void updatePose(const turtlesim::PoseConstPtr &currentPose) {
  current.x = currentPose->x;
  current.y = currentPose->y;
  current.theta = currentPose->theta;
}
 
int main(int argc, char **argv) {
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;

  ros::param::param<double>("~Goal_x", goal_x, 0);
  ros::param::param<double>("~Precision_x", precision_x, 0);
  ros::param::param<double>("~K_l", k_l, 0);

  setup();  
 
  // Subscribe to the robot's pose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new pose is received, update the robot's pose.
  ros::Subscriber currentPoseSub =
    node.subscribe("turtle1/pose", 0, updatePose);
 
  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  ros::Publisher velocityPub =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
 
  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate loop_rate(10); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok() && !goal_x_reached) {
 
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    setVelocity();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
 
    // Print the output to the console
    ROS_INFO("Current x = %f\n"
              "Desired x = %f\n"
              "Distance to goal_x = %f m\n"
              "Linear Velocity (x) = %f m/s\n",
              current.x, desired.x, getDistanceTogoal_x(), velCommand.linear.x);
 
    // Sleep as long as we need to to make sure that we have a frequency of
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}