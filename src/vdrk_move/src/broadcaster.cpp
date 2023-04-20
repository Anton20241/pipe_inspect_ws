#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
   ros::init(argc, argv, "broadcaster");
   ros::NodeHandle n;
   ros::Rate r(100);
   tf::TransformBroadcaster broadcaster;
   
   while(n.ok()){
      
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.076, 0, 0.094)),
            ros::Time::now(), "turtle_back_frame", "turtle_back_camera_frame"));

      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476), tf::Vector3(-0.21, 0, 0.25)),
            ros::Time::now(), "turtle_front_frame", "aruco_frame"));
            
      r.sleep();
   }
}