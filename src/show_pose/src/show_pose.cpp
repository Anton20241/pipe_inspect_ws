#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

void modelStateCallback(const gazebo_msgs::ModelStates& msg){

  ROS_INFO("Got:\n"
    "pos.linear: x= %.5f y= %.5f z= %.5f\n"
    "pos.angular x= %.5f y= %.5f z= %.5f w= %.5f\n",
    msg.pose[1].position.x, msg.pose[1].position.y, msg.pose[1].position.z,
    msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w);

  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_pose_sub");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 0, &modelStateCallback);
  ros::spin();
  return 0;
};
