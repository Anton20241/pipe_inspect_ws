#include <ros/ros.h>
#include <rosbag/bag.h>
#include "send_estimate_data/Poses.h"

rosbag::Bag arucoResData;

void arucoPosesCatchAndWrite(const send_estimate_data::Poses& posesMsg){
  
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
            posesMsg.estimatePose.pose.position.x,
            posesMsg.estimatePose.pose.position.y,
            posesMsg.estimatePose.pose.position.z,

            posesMsg.truePose.pose.position.x,
            posesMsg.truePose.pose.position.y,
            posesMsg.truePose.pose.position.z);
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