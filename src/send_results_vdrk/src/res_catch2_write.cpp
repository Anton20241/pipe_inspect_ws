#include <ros/ros.h>
#include <rosbag/bag.h>
#include "send_results_vdrk/Poses4.h"

rosbag::Bag arucoResData;

void arucoPosesCatchAndWrite(const send_results_vdrk::Poses4& posesMsg){
  
  ROS_INFO_STREAM("!!!--arucoPosesCatch2Write catched new msg from arucoResDataBagTopic--!!!");
  arucoResData.write("arucoResDataBagTopic", ros::Time::now(), posesMsg);
  ROS_INFO("arucoPosesCatch2Write writed the next data to arucoPosesCatch2Write.bag:\n"
            "---------------------------------------------------- \n"
            "estimatePose.pose.position.x = %.5f \n"
            "estimatePose.pose.position.y = %.5f \n"
            "estimatePose.pose.position.z = %.5f \n"
            "---------------------------------------------------- \n"
            "truePose.pose.position.x = %.5f \n"
            "truePose.pose.position.y = %.5f \n"
            "truePose.pose.position.z = %.5f \n"
            "---------------------------------------------------- \n",
            posesMsg.estimateArCamPose.pose.position.x,
            posesMsg.estimateArCamPose.pose.position.y,
            posesMsg.estimateArCamPose.pose.position.z,

            posesMsg.trueArCamPose.pose.position.x,
            posesMsg.trueArCamPose.pose.position.y,
            posesMsg.trueArCamPose.pose.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arucoPosesCatch2Write");
  ROS_INFO_STREAM("!!!--arucoPosesCatch2Write is ready--!!!");
  ros::NodeHandle n;
  arucoResData.open("arucoPosesCatch2Write.bag", rosbag::bagmode::Write);
  ros::Subscriber sub = n.subscribe("arucoResDataBagTopic", 0, arucoPosesCatchAndWrite);
  ros::spin();
  arucoResData.close();
  return 0;
}