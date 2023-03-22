#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
   ros::init(argc, argv, "odom_cam_br");
   ros::NodeHandle n;
   ros::Rate r(100);
   tf::TransformBroadcaster broadcaster;
   	
   while(n.ok()){
      broadcaster.sendTransform(
         tf::StampedTransform(
            tf::Transform(tf::Quaternion(-1, 0, 0, 1), tf::Vector3(0, -2, 0.25)),
            ros::Time::now(), "odom", "stereo_gazebo_left_camera_optical_frame"));
      r.sleep();
   }
}

