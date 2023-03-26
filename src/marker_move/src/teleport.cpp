#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

gazebo_msgs::ModelStates modelSet;               // задаваемое имя поза и скорость
geometry_msgs::Pose      currentPose;            // текущее положение маркера

bool getCurrentOdomPoseCallback = false;

ros::V_string model_name;

double position_x = 0;
double position_y = 0;
double position_z = 0;

double orientation_x = 0;
double orientation_y = 0;
double orientation_z = 0;
double orientation_w = 0;

void setModelName() {
  //modelSet.name = model_name;
}

void setPose() {
  modelSet.pose[1].position.x = position_x;
  modelSet.pose[1].position.y = position_y;
  modelSet.pose[1].position.z = position_z;

  modelSet.pose[1].orientation.x = orientation_x;
  modelSet.pose[1].orientation.y = orientation_y;
  modelSet.pose[1].orientation.z = orientation_z;
  modelSet.pose[1].orientation.w = orientation_w;
}

void setVelocity() {
  modelSet.twist[1].linear.x = 0.0;
  modelSet.twist[1].linear.y = 0.0;
  modelSet.twist[1].linear.z = 0.0;
  modelSet.twist[1].angular.x = 0.0;
  modelSet.twist[1].angular.y = 0.0;
  modelSet.twist[1].angular.z = 0.0;
}

void getCurrentArucoOdomPose(const gazebo_msgs::ModelStates& modelGazeboMsg) {
  currentPose.position.x = modelGazeboMsg.pose[1].position.x;
  currentPose.position.y = modelGazeboMsg.pose[1].position.y;
  currentPose.position.z = modelGazeboMsg.pose[1].position.z;

  currentPose.orientation.w = modelGazeboMsg.pose[1].orientation.w;
  currentPose.orientation.x = modelGazeboMsg.pose[1].orientation.x;
  currentPose.orientation.y = modelGazeboMsg.pose[1].orientation.y;
  currentPose.orientation.z = modelGazeboMsg.pose[1].orientation.z;

  getCurrentOdomPoseCallback = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "teleport");
  ros::NodeHandle node;

  ros::param::param<std::string>("~Model_name", model_name, "");
  ros::param::param<double>("~Position_x", position_x, 0);
  ros::param::param<double>("~Position_y", position_y, 0);
  ros::param::param<double>("~Position_z", position_z, 0);
  ros::param::param<double>("~Orientation_x", orientation_x, 0);
  ros::param::param<double>("~Orientation_y", orientation_y, 0);
  ros::param::param<double>("~Orientation_z", orientation_z, 0);
  ros::param::param<double>("~Orientation_w", orientation_w, 0);

  ros::Subscriber currentArucoOdomPoseSub =
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);

  ros::Publisher Cmd2ModelPub = 
    node.advertise<gazebo_msgs::ModelStates>("/gazebo/set_link_state", 0);
  
  setModelName();
  setVelocity();
  setPose();

  Cmd2ModelPub.publish(modelSet);

  while(ros::ok && !getCurrentOdomPoseCallback){
    ros::spinOnce();
  }
  
  ROS_INFO("\n"
            "Position_x = %.5f\n"
            "Position_y = %.5f\n"
            "Position_z = %.5f\n"
            "Orientation_x = %.5f\n"
            "Orientation_y = %.5f\n"
            "Orientation_z = %.5f\n"
            "Orientation_w = %.5f\n",
            currentPose.position.x, currentPose.position.y, currentPose.position.z,
            currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
            currentPose.orientation.w);

  return 0;
}