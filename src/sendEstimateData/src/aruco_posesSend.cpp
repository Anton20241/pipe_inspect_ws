#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <turtlesim/Pose.h>
#include "sendEstimateData/Poses.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

#define MEASURE_STEP 0.01                                         // [м] = 1 см - шаг измерения положения маркера

//сообщение в топик на запись данных
sendEstimateData::Poses arucoPoses2Write;
ros::Publisher arucoResDataBagPub;

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее оценочное положение маркера относительно камеры

geometry_msgs::Pose currentArucoOdomPose;                         // текущее фактическое положение маркера относительно ГСК
geometry_msgs::PoseStamped currentArucoCameraPose;                // текущее фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped prevArucoCameraPose;                   // предыдущее фактическое положение маркера относительно камеры
geometry_msgs::Pose tmp_ArucoOdomPose;                            // временная переменная

bool arucoStartPose = true;                                       // маркер еще не двигался

bool allCallbacksCall = false;                                    // все колбеки вызваны
bool getEstimateCurrentArucoCameraPoseCallback = false;
bool getCurrentArucoOdomPoseCallback = false;

void setup() {
  estimateCurrentArucoCameraPose.pose.position.x = 0.0;
  estimateCurrentArucoCameraPose.pose.position.y = 0.0;
  estimateCurrentArucoCameraPose.pose.position.z = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.w = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.x = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.y = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.z = 0.0;
}

// получаем текущую оценку положения маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

// получаем текущее фактическое положение моделей относительно ГСК
void getCurrentArucoOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  currentArucoOdomPose.position.x = arucoGazeboMsg.pose[1].position.x;
  currentArucoOdomPose.position.y = arucoGazeboMsg.pose[1].position.y;
  currentArucoOdomPose.position.z = arucoGazeboMsg.pose[1].position.z;

  currentArucoOdomPose.orientation.w = arucoGazeboMsg.pose[1].orientation.w;
  currentArucoOdomPose.orientation.x = arucoGazeboMsg.pose[1].orientation.x;
  currentArucoOdomPose.orientation.y = arucoGazeboMsg.pose[1].orientation.y;
  currentArucoOdomPose.orientation.z = arucoGazeboMsg.pose[1].orientation.z;

  getCurrentArucoOdomPoseCallback = true;
}

// получаем факт положение маркера относительно камеры 
void transformPose(const tf::TransformListener& listener){
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("camera_frame", "aruco_marker_frame", ros::Time(0), transform);
    
    currentArucoCameraPose.header.frame_id = "camera_frame";
    currentArucoCameraPose.pose.position.x = transform.getOrigin().x();
    currentArucoCameraPose.pose.position.y = transform.getOrigin().y();
    currentArucoCameraPose.pose.position.z = transform.getOrigin().z();
    currentArucoCameraPose.pose.orientation.w = transform.getRotation().getW();
    currentArucoCameraPose.pose.orientation.x = transform.getRotation().getX();
    currentArucoCameraPose.pose.orientation.y = transform.getRotation().getY();
    currentArucoCameraPose.pose.orientation.z = transform.getRotation().getZ();
    
    ROS_INFO("\n"
              "<--------------odom:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->\n"
              "<------camera_frame:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->",
      currentArucoOdomPose.position.x, 
      currentArucoOdomPose.position.y,    currentArucoOdomPose.position.z,
      currentArucoOdomPose.orientation.w, currentArucoOdomPose.orientation.x,
      currentArucoOdomPose.orientation.y, currentArucoOdomPose.orientation.z,
      
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

// расстояние между точками
double getDistance(const double t1, const double t2) {                
  return t1 - t2;
}

void setArucoPoses2Write(){
  arucoPoses2Write.estimatePose = estimateCurrentArucoCameraPose;    //оценочное положение
  arucoPoses2Write.truePose     = currentArucoCameraPose;            //фактическое положение
}

// записываем фактическое и оценочное положение маркера
void writeArucoPoseData2Bag(){                                        

  if (arucoStartPose){
    currentArucoCameraPose = prevArucoCameraPose;
    arucoStartPose = false;
  }

  geometry_msgs::Vector3 MarkerOffset;
  
  //фактическое перемещение маркера
  MarkerOffset.x = abs(getDistance(currentArucoCameraPose.pose.position.x, prevArucoCameraPose.pose.position.x));
  MarkerOffset.y = abs(getDistance(currentArucoCameraPose.pose.position.y, prevArucoCameraPose.pose.position.y));
  MarkerOffset.z = abs(getDistance(currentArucoCameraPose.pose.position.z, prevArucoCameraPose.pose.position.z));
  double MarkerSpaceOffset = std::sqrt(std::pow(MarkerOffset.x, 2) + std::pow(MarkerOffset.y, 2) + std::pow(MarkerOffset.z, 2));
  prevArucoCameraPose = currentArucoCameraPose;
  
  if (MarkerSpaceOffset > MEASURE_STEP) {
    setArucoPoses2Write();
    arucoResDataBagPub.publish(arucoPoses2Write);
    ROS_INFO("\n"
              "######################################################\n"
              "!!! SEND POSES DATA !!!\n"
              "estimateCurrentArucoCameraPose.pose.position.x = %.5f \n"
              "estimateCurrentArucoCameraPose.pose.position.y = %.5f \n"
              "estimateCurrentArucoCameraPose.pose.position.z = %.5f \n"
              "------------------------------------------------------\n"
              "currentArucoCameraPose.pose.position.x = %.5f \n"
              "currentArucoCameraPose.pose.position.y = %.5f \n"
              "currentArucoCameraPose.pose.position.z = %.5f \n"
              "######################################################\n\n",
              estimateCurrentArucoCameraPose.pose.position.x,
              estimateCurrentArucoCameraPose.pose.position.y,
              estimateCurrentArucoCameraPose.pose.position.z,

              currentArucoCameraPose.pose.position.x,
              currentArucoCameraPose.pose.position.y,
              currentArucoCameraPose.pose.position.z);
  }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "aruco_posesSend");

  ros::NodeHandle node;
  tf::TransformListener listener;

  setup();

  ros::Subscriber arucoCameraPoseSub =
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);

  ros::Subscriber arucoGazeboPoseSub =
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);

  arucoResDataBagPub =
    node.advertise<sendEstimateData::Poses>("arucoResDataBagTopic", 0);

  ros::Rate loop_rate(100);

  while (ros::ok()) {

    ros::spinOnce();
    
    allCallbacksCall = getEstimateCurrentArucoCameraPoseCallback &&
                          getCurrentArucoOdomPoseCallback;

    if (!allCallbacksCall) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;
    getCurrentArucoOdomPoseCallback           = false;

    transformPose(listener);
    writeArucoPoseData2Bag();

    loop_rate.sleep();
  }

  return 0;
}