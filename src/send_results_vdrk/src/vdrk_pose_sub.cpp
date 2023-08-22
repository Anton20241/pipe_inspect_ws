#include <ros/ros.h>
#include <rosbag/bag.h>
#include "send_results_vdrk/VdrkPose.h"

rosbag::Bag vdrkPose;

void vdrkPoseHandler(const send_results_vdrk::VdrkPose& vdrkPoseMsg){
  vdrkPose.write("vdrkPoseTopic", ros::Time::now(), vdrkPoseMsg);
  printf("vdrkPoseHandler writed the next data to vdrkPose.bag:\n");
  printf("######################################################\n");
  printf("[текущее оценочное положение _маркера_ относительно _камеры_]\n");
  printf("estCrntArCamPose.pose.position.x    = %f\n", vdrkPoseMsg.estCrntArCamPose.pose.position.x);
  printf("estCrntArCamPose.pose.position.y    = %f\n", vdrkPoseMsg.estCrntArCamPose.pose.position.y);
  printf("estCrntArCamPose.pose.position.z    = %f\n", vdrkPoseMsg.estCrntArCamPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно _камеры_]\n");
  printf("crntArCamPose.pose.position.x       = %f\n", vdrkPoseMsg.crntArCamPose.pose.position.x);
  printf("crntArCamPose.pose.position.y       = %f\n", vdrkPoseMsg.crntArCamPose.pose.position.y);
  printf("crntArCamPose.pose.position.z       = %f\n", vdrkPoseMsg.crntArCamPose.pose.position.z);
  printf("\n######################################################\n");
  printf("[текущее оценочное положение _маркера_ относительно ГСК]\n");
  printf("estCrntArOdomPose.pose.position.x   = %f\n", vdrkPoseMsg.estCrntArOdomPose.pose.position.x);
  printf("estCrntArOdomPose.pose.position.y   = %f\n", vdrkPoseMsg.estCrntArOdomPose.pose.position.y);
  printf("estCrntArOdomPose.pose.position.z   = %f\n", vdrkPoseMsg.estCrntArOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно ГСК]\n");
  printf("crntArOdomPose.pose.position.x      = %f\n", vdrkPoseMsg.crntArOdomPose.pose.position.x);
  printf("crntArOdomPose.pose.position.y      = %f\n", vdrkPoseMsg.crntArOdomPose.pose.position.y);
  printf("crntArOdomPose.pose.position.z      = %f\n", vdrkPoseMsg.crntArOdomPose.pose.position.z);
  printf("\n######################################################\n");
  printf("[текущее оценочное положение _камеры_ относительно ГСК]\n");
  printf("estCrntCamOdomPose.pose.position.x  = %f\n", vdrkPoseMsg.estCrntCamOdomPose.pose.position.x);
  printf("estCrntCamOdomPose.pose.position.y  = %f\n", vdrkPoseMsg.estCrntCamOdomPose.pose.position.y);
  printf("estCrntCamOdomPose.pose.position.z  = %f\n", vdrkPoseMsg.estCrntCamOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно ГСК]\n");
  printf("crntCamOdomPose.pose.position.x     = %f\n", vdrkPoseMsg.crntCamOdomPose.pose.position.x);
  printf("crntCamOdomPose.pose.position.y     = %f\n", vdrkPoseMsg.crntCamOdomPose.pose.position.y);
  printf("crntCamOdomPose.pose.position.z     = %f\n", vdrkPoseMsg.crntCamOdomPose.pose.position.z);
  printf("######################################################\n\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vdrk_pose_sub");
  ROS_INFO_STREAM("vdrk_pose_sub is ready");
  ros::NodeHandle n;
  vdrkPose.open("vdrkPose.bag", rosbag::bagmode::Write);
  ros::Subscriber vdrkPoseSub = n.subscribe("vdrkPoseTopic", 0, vdrkPoseHandler);
  ros::spin();
  vdrkPose.close();
  return 0;
}