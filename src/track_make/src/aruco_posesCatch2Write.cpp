#include <ros/ros.h>
#include <rosbag/bag.h>
#include "track_make/Poses.h"

rosbag::Bag arucoResData;

void arucoPosesCatchAndWrite(const track_make::Poses& posesMsg){
  
  ROS_INFO_STREAM("!!!--ArucoPosesCatch2Write catched new msg from arucoResDataBagTopic--!!!\n");
  arucoResData.write("arucoResDataBagTopic", ros::Time::now(), posesMsg);
  ROS_INFO("ArucoPosesCatch2Write writed the next data to ArucoPosesCatch2Write.bag:\n"
            "---------------------------------------------------- \n"
            "posesMsg.estimatePose.pose.position.x = %.5f \n"
            "posesMsg.estimatePose.pose.position.y = %.5f \n"
            "posesMsg.estimatePose.pose.position.z = %.5f \n"
            "---------------------------------------------------- \n"
            "posesMsg.truePose.pose.position.x = %.5f \n"
            "posesMsg.truePose.pose.position.y = %.5f \n"
            "posesMsg.truePose.pose.position.z = %.5f \n"
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
  ROS_INFO_STREAM("!!!--ArucoPosesCatch2Write is ready--!!!");
  ros::NodeHandle n;
  arucoResData.open("ArucoPosesCatch2Write.bag", rosbag::bagmode::Write);
  ros::Subscriber sub = n.subscribe("arucoResDataBagTopic", 100, arucoPosesCatchAndWrite);
  ros::spin();
  arucoResData.close();
  return 0;
}