#include "img_from_cam_pub.hpp"

using namespace cv;

ImgFromCamPubNode::ImgFromCamPubNode()
  : nh("~"), m_image_transport(nh), vid(0) 
{
  m_image_publisher = m_image_transport.advertise("imgFromCamTopic", 1);
  assert(vid.isOpened());
  namedWindow("frame", WINDOW_AUTOSIZE);
}

void ImgFromCamPubNode::getAndSendImg(){
  if (!vid.read(inImage)) return;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time();
  out_msg.encoding = sensor_msgs::image_encodings::RGB8;
  out_msg.image = inImage;
  m_image_publisher.publish(out_msg.toImageMsg());
}

ImgFromCamPubNode::~ImgFromCamPubNode(){;}