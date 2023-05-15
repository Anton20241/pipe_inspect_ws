#include <ros/ros.h>
#include <rosbag/bag.h>
#include "send_results_vdrk/Poses4.h"

rosbag::Bag arucoResData;
rosbag::Bag cameraResData;

void arucoPosesHandler(const send_results_vdrk::Poses4& posesMsg){
  
  arucoResData.write("arucoPosesTopic", ros::Time::now(), posesMsg);

  printf("arucoPosesHandler writed the next data to arucoPoses.bag:\n");
  printf("######################################################\n");
  printf("[текущее оценочное положение _маркера_ относительно 0 точки]\n");
  printf("estimateZeroPointPose.pose.position.x = %f\n", posesMsg.estimateZeroPointPose.pose.position.x);
  printf("estimateZeroPointPose.pose.position.y = %f\n", posesMsg.estimateZeroPointPose.pose.position.y);
  printf("estimateZeroPointPose.pose.position.z = %f\n", posesMsg.estimateZeroPointPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно 0 точки]\n");
  printf("trueZeroPointPose.pose.position.x     = %f\n", posesMsg.trueZeroPointPose.pose.position.x);
  printf("trueZeroPointPose.pose.position.y     = %f\n", posesMsg.trueZeroPointPose.pose.position.y);
  printf("trueZeroPointPose.pose.position.z     = %f\n", posesMsg.trueZeroPointPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее оценочное положение _маркера_ относительно ГСК]\n");
  printf("estimateOdomPose.pose.position.x      = %f\n", posesMsg.estimateOdomPose.pose.position.x);
  printf("estimateOdomPose.pose.position.y      = %f\n", posesMsg.estimateOdomPose.pose.position.y);
  printf("estimateOdomPose.pose.position.z      = %f\n", posesMsg.estimateOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно ГСК]\n");
  printf("trueOdomPose.pose.position.x          = %f\n", posesMsg.trueOdomPose.pose.position.x);
  printf("trueOdomPose.pose.position.y          = %f\n", posesMsg.trueOdomPose.pose.position.y);
  printf("trueOdomPose.pose.position.z          = %f\n", posesMsg.trueOdomPose.pose.position.z);
  printf("\n######################################################\n");
}

void cameraPosesHandler(const send_results_vdrk::Poses4& posesMsg){
  
  cameraResData.write("cameraPosesTopic", ros::Time::now(), posesMsg);

  printf("cameraPosesHandler writed the next data to cameraPoses.bag:\n");
  printf("######################################################\n");
  printf("[текущее оценочное положение _камеры_ относительно 0 точки]\n");
  printf("estimateZeroPointPose.pose.position.x = %f\n", posesMsg.estimateZeroPointPose.pose.position.x);
  printf("estimateZeroPointPose.pose.position.y = %f\n", posesMsg.estimateZeroPointPose.pose.position.y);
  printf("estimateZeroPointPose.pose.position.z = %f\n", posesMsg.estimateZeroPointPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно 0 точки]\n");
  printf("trueZeroPointPose.pose.position.x     = %f\n", posesMsg.trueZeroPointPose.pose.position.x);
  printf("trueZeroPointPose.pose.position.y     = %f\n", posesMsg.trueZeroPointPose.pose.position.y);
  printf("trueZeroPointPose.pose.position.z     = %f\n", posesMsg.trueZeroPointPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее оценочное положение _камеры_ относительно ГСК]\n");
  printf("estimateOdomPose.pose.position.x      = %f\n", posesMsg.estimateOdomPose.pose.position.x);
  printf("estimateOdomPose.pose.position.y      = %f\n", posesMsg.estimateOdomPose.pose.position.y);
  printf("estimateOdomPose.pose.position.z      = %f\n", posesMsg.estimateOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно ГСК]\n");
  printf("trueOdomPose.pose.position.x          = %f\n", posesMsg.trueOdomPose.pose.position.x);
  printf("trueOdomPose.pose.position.y          = %f\n", posesMsg.trueOdomPose.pose.position.y);
  printf("trueOdomPose.pose.position.z          = %f\n", posesMsg.trueOdomPose.pose.position.z);
  printf("\n######################################################\n");
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "PosesCatch2Write");
  ROS_INFO_STREAM("!!!--posesCatch2Write is ready--!!!");
  ros::NodeHandle n;

  arucoResData.open("arucoPoses.bag",   rosbag::bagmode::Write);
  cameraResData.open("cameraPoses.bag", rosbag::bagmode::Write);

  ros::Subscriber arucoPosesSub   = n.subscribe("arucoPosesTopic", 0, arucoPosesHandler);
  ros::Subscriber cameraPosesSub  = n.subscribe("cameraPosesTopic", 0, cameraPosesHandler);

  ros::spin();
  arucoResData.close();
  cameraResData.close();
  
  return 0;
}