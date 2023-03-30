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
            tf::Transform(tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476), tf::Vector3(0, -2, 0.25)),
            ros::Time::now(), "odom", "camera_frame"));
      r.sleep();
   }
}