#ifndef IMGFROMCAMPUB_HPP
#define IMGFROMCAMPUB_HPP

#include <ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<opencv2/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

class ImgFromCamPubNode{
public:
  ImgFromCamPubNode();
  void getAndSendImg();
  ~ImgFromCamPubNode();
  
private:
  cv::VideoCapture vid;
  cv::Mat inImage;
  ros::NodeHandle nh;
  image_transport::ImageTransport m_image_transport;
  image_transport::Publisher m_image_publisher;
};

#endif  // IMGFROMCAMPUB_HPP