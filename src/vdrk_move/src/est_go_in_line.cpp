/*
  Программа предназначена для перемещения ВДРК по прямой линии.
  В качестве аргументов - скорость перемещения роботов.
  Зная расстояние от маркера до камеры, 
  сперва перемещается передний робот до заданного расстояния между роботами,
  затем его догоняет задний робот.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/asio.hpp>

#define MIN_ROBOTS_DIST       4.0                                 // минимальное  расстояние между камерой и маркером   [м]
#define MAX_ROBOTS_DIST       6.0                                 // максимальное расстояние между камерой и маркером   [м]
#define ROBOTS_DIST_PRECISION 0.00001                             // точность оценочного взаимного расположения роботов [м]

geometry_msgs::PoseStamped estCrntArCamPose;                      // текущее оценочное положение _маркера_ относительно _камеры_
geometry_msgs::Twist       velVdrkMsg;                            // сообщение скорости для роботов [м/с]
ros::Publisher             velCmdCamPub;                          // паблишер  скорости для камеры
ros::Publisher             velCmdArPub;                           // паблишер  скорости для маркера
ros::Subscriber            estCrntArCamPoseSub;

double vel4VdrkFromUser       = 0;                                // скорость для роботов от пользователя [м/с]
double estCrntRobotsDist      = 0;                                // расстояние между маркером и камерой
bool camera_is_stop           = true;
bool marker_is_stop           = true;
bool getEstCrntArCamPose      = false;

void setStopVdrk(){
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;
}

void marker_go(){
  if ((ROBOTS_DIST_PRECISION < (MAX_ROBOTS_DIST - estCrntRobotsDist)) && camera_is_stop) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stop       = false;
  } else {
    setStopVdrk();
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stop       = true;
    if (marker_is_stop && camera_is_stop){
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
}

void camera_go(){
  if (((estCrntRobotsDist - MIN_ROBOTS_DIST) > ROBOTS_DIST_PRECISION) && marker_is_stop) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stop       = false;
  } else {
    setStopVdrk();
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stop       = true;
    if (marker_is_stop && camera_is_stop){
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
}

void go_vdrk_in_line(){
  estCrntRobotsDist = std::sqrt(std::pow(estCrntArCamPose.pose.position.x, 2) + 
                                std::pow(estCrntArCamPose.pose.position.y, 2) + 
                                std::pow(estCrntArCamPose.pose.position.z, 2));
  ROS_INFO("\nestCrntRobotsDist = %.5f\n", estCrntRobotsDist);
  marker_go();
  camera_go();
}

// получаем текущее оценочное положение _маркера_ относительно _камеры_
void getEstCrntArCamPoseHandler(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estCrntArCamPose    = arucoCameraMsg;
  getEstCrntArCamPose = true;
}

void setup(ros::NodeHandle& node) {
  estCrntArCamPoseSub  = node.subscribe("/aruco_single/pose", 0, getEstCrntArCamPoseHandler);
  velCmdCamPub         = node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);
  velCmdArPub          = node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel",  0);
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "est_go_in_line");
  ros::NodeHandle node;
  ros::param::param<double>("~Velocity", vel4VdrkFromUser, 0);
  setup(node);
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (!getEstCrntArCamPose) continue;
    getEstCrntArCamPose = false;
    go_vdrk_in_line();
    loop_rate.sleep();
  }
  return 0;
}