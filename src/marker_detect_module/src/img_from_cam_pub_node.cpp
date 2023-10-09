#include "img_from_cam_pub.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "img_from_cam_pub_node");
  ImgFromCamPubNode Node;

  int32_t looprate = 30; // hz
  ros::Rate loop_rate(looprate);
  

  while (ros::ok()) {
    Node.getAndSendImg();
    loop_rate.sleep();
  }
}