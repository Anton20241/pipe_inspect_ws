#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
   ros::init(argc, argv, "camera_frame_broadcaster");
   ros::NodeHandle n;
   ros::Rate r(30);
   tf::TransformBroadcaster broadcaster;
   	
   while(n.ok()){
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.15)),
            ros::Time::now(), "camera_link_base", "camera_link"));
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476), tf::Vector3(0, 0, 0)),
            ros::Time::now(), "camera_link", "camera_link_optical"));
      r.sleep();
   }
}