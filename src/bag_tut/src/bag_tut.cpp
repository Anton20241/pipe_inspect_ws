#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "testBug");
  ros::NodeHandle n;

    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Write);

    std_msgs::String str;
    str.data = std::string("foo");

    std_msgs::Int32 i;
    i.data = 42;

    bag.write("chatter", ros::TIME_MIN, str);
    bag.write("numbers", ros::TIME_MIN, i);

    bag.close();

  ROS_INFO("ros::Time::now() = %.50f, ros::TIME_MIN = %.50f\n", ros::Time::now(), ros::TIME_MIN);
  return 0;
}

