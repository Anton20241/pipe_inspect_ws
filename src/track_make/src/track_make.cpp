#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include<opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/aruco.hpp>
#include<opencv2/calib3d.hpp>

using namespace cv;

// факт положение маркера
ros::Publisher arucoTrajectoryTruePub;
nav_msgs::Path arucoTrajectoryTruePath;

// оценочное положение маркера
ros::Publisher arucoTrajectoryEstimatePub;
nav_msgs::Path arucoTrajectoryEstimatePath;

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее оценочное положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoCameraPose;                // текущее фактическое положение маркера относительно камеры

bool getEstimateCurrentArucoCameraPoseCallback = false;

// получаем текущее оценочное положение маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

// получаем текущее фактическое положение маркера относительно камеры
void transformPose(const tf::TransformListener& listener){
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("camera_frame", "aruco_frame", ros::Time(0), transform);
    
    currentArucoCameraPose.header.frame_id = "camera_frame";
    currentArucoCameraPose.pose.position.x = transform.getOrigin().x();
    currentArucoCameraPose.pose.position.y = transform.getOrigin().y();
    currentArucoCameraPose.pose.position.z = transform.getOrigin().z();
    currentArucoCameraPose.pose.orientation.w = transform.getRotation().getW();
    currentArucoCameraPose.pose.orientation.x = transform.getRotation().getX();
    currentArucoCameraPose.pose.orientation.y = transform.getRotation().getY();
    currentArucoCameraPose.pose.orientation.z = transform.getRotation().getZ();
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void sendArucoTruePose() {
  arucoTrajectoryTruePath.header.frame_id = "camera_frame";
  arucoTrajectoryTruePath.poses.push_back(currentArucoCameraPose);
  arucoTrajectoryTruePub.publish(arucoTrajectoryTruePath);
}

void sendArucoEstimatePose() {
  arucoTrajectoryEstimatePath.header.frame_id = "camera_frame";
  arucoTrajectoryEstimatePath.poses.push_back(estimateCurrentArucoCameraPose);
  arucoTrajectoryEstimatePub.publish(arucoTrajectoryEstimatePath);
}



int main(int argc, char **argv) {

  ros::init(argc, argv, "track_make");
  ros::NodeHandle node;
  tf::TransformListener listener;

  ros::Subscriber arucoCameraPoseSub =
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);

  arucoTrajectoryTruePub =
    node.advertise<nav_msgs::Path>("arucoTrajectoryTruePub", 1);

  arucoTrajectoryEstimatePub =
    node.advertise<nav_msgs::Path>("arucoTrajectoryEstimatePub", 1);

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    char character = waitKey(1000 / 30);

    switch (character)
    {
      case 8:
        arucoTrajectoryTruePath.poses.clear();
        arucoTrajectoryEstimatePath.poses.clear();
        break;
      default:
        break;
    }
    ros::spinOnce();
    
    if (!getEstimateCurrentArucoCameraPoseCallback) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;

    transformPose(listener);
    sendArucoTruePose();
    sendArucoEstimatePose();

    loop_rate.sleep();
  }

  return 0;
}