#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

void modelStatecallback(const gazebo_msgs::ModelStates& msg){

  ROS_INFO("Got:\n"
    "1) pos.linear: x= %f y= %f z= %f\n"
    "2) pos.angular x= %f y= %f z= %f w= %f\n",
    msg.pose[1].position.x, msg.pose[1].position.y, msg.pose[1].position.z,
    msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w);

  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_pose_sub");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &modelStatecallback);
  ros::spin();
  return 0;
};
