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

// tf::Transform(tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476), tf::Vector3(0, 0, 0)),

   //  <link name="camera_link_optical">
   //    <pose>0 0 0 -pi/2 0 -pi/2</pose>
   //  </link>

   //  <joint name="camera_optical_joint" type="fixed">
   //      <child>camera_link_optical</child>
   //      <parent>camera_link</parent>
   //  </joint>


   //  <joint name="camera_joint" type="fixed">
   //      <child>camera_link</child>
   //      <parent>camera_link_base</parent>
   //  </joint>