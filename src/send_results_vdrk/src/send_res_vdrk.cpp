#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include "send_results_vdrk/Poses4.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

#define MEASURE_STEP 0.05                                         // [м] = 5 см - шаг измерения положения маркера

// сообщение в топик на запись данных
send_results_vdrk::Poses4 res2Write;
// паблишер в топик на запись данных
ros::Publisher arucoResDataBagPub;

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее      оценочное положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoCameraPose;                // текущее    фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped prevArucoCameraPose;                   // предыдущее фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoOdomPose;                  // текущее    фактическое положение маркера относительно ГСК
geometry_msgs::PoseStamped estimateCurrentArucoOdomPose;          // текущее      оценочное положение маркера относительно ГСК

bool allCallbacksCall = false;                                    // все колбеки вызваны
bool getEstimateCurrentArucoCameraPoseCallback = false;
bool getCurrentArucoOdomPoseCallback = false;


// получаем текущее оценочное положение маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

// получаем текущее фактическое положение маркера относительно ГСК
void getCurrentArucoOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  currentArucoOdomPose.pose.position.x = arucoGazeboMsg.pose[1].position.x;
  currentArucoOdomPose.pose.position.y = arucoGazeboMsg.pose[1].position.y;
  currentArucoOdomPose.pose.position.z = arucoGazeboMsg.pose[1].position.z;

  currentArucoOdomPose.pose.orientation.w = arucoGazeboMsg.pose[1].orientation.w;
  currentArucoOdomPose.pose.orientation.x = arucoGazeboMsg.pose[1].orientation.x;
  currentArucoOdomPose.pose.orientation.y = arucoGazeboMsg.pose[1].orientation.y;
  currentArucoOdomPose.pose.orientation.z = arucoGazeboMsg.pose[1].orientation.z;

  getCurrentArucoOdomPoseCallback = true;
}

// получаем текущее фактическое положение маркера относительно камеры
void getCurrentArucoCameraPose(const tf::TransformListener& listener){
  tf::StampedTransform transform;
  try{
    currentArucoOdomPose.header.frame_id = "odom";
    currentArucoOdomPose.header.stamp    = ros::Time();

    listener.transformPose("camera_link_optical", currentArucoOdomPose, currentArucoCameraPose);
    std::cout << std::endl;
    ROS_INFO("\n"
              "<---------------------odom:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->\n"
              "<------camera_link_optical:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->",
      currentArucoOdomPose.pose.position.x, 
      currentArucoOdomPose.pose.position.y,    currentArucoOdomPose.pose.position.z,
      currentArucoOdomPose.pose.orientation.w, currentArucoOdomPose.pose.orientation.x,
      currentArucoOdomPose.pose.orientation.y, currentArucoOdomPose.pose.orientation.z,
      
      currentArucoCameraPose.pose.position.x, 
      currentArucoCameraPose.pose.position.y,    currentArucoCameraPose.pose.position.z,
      currentArucoCameraPose.pose.orientation.w, currentArucoCameraPose.pose.orientation.x,
      currentArucoCameraPose.pose.orientation.y, currentArucoCameraPose.pose.orientation.z);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}


int main(int argc, char **argv){

  ros::init(argc, argv, "send_res_vdrk");
  ros::NodeHandle node;
  tf::TransformListener listener;

  // текущее оценочное положение маркера относительно камеры
  ros::Subscriber arucoCameraPoseSub =                                             
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);

  // текущее фактическое положение маркера относительно ГСК
  ros::Subscriber arucoGazeboPoseSub =                                             
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);

  arucoResDataBagPub =
    node.advertise<send_results_vdrk::Poses4>("arucoResDataBagTopic", 0);

  ros::Rate loop_rate(30);

  while (ros::ok()){

    ros::spinOnce();

    allCallbacksCall = getEstimateCurrentArucoCameraPoseCallback &&
                          getCurrentArucoOdomPoseCallback;
    if (!allCallbacksCall) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;
    getCurrentArucoOdomPoseCallback           = false;

    getCurrentArucoCameraPose(listener);                                           // текущее фактическое положение маркера относительно камеры
    // getEstimeteCurrentArucoCameraPose();                                           // текущее   оценочное положение маркера относительно ГСК

    // writeArucoPoseData2Bag();

    loop_rate.sleep();
  }

  return 0;
}