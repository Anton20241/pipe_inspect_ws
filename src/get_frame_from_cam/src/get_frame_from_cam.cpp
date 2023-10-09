#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include<opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_frame_from_cam_node");
  ROS_INFO("get_frame_from_cam_node is running");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  Mat frame;
  VideoCapture vid(0);
  assert(vid.isOpened());
  namedWindow("Camera calibrate", WINDOW_AUTOSIZE);

  while (ros::ok())
  {
    if (!vid.read(frame)) continue;
    imshow("Camera calibrate", frame);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}