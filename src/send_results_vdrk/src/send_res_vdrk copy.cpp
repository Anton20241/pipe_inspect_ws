#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include "send_results_vdrk/Poses4.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

#define MEASURE_STEP 0.05                                          // [м] = 5 см - шаг измерения положения маркера

// сообщение в топик на запись данных
send_results_vdrk::Poses4 res2Write;
// паблишер в топик на запись данных
ros::Publisher arucoResDataBagPub;

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее      оценочное положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoCameraPose;                // текущее    фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped prevArucoCameraPose;                   // предыдущее фактическое положение маркера относительно камеры

geometry_msgs::PoseStamped currentArucoOdomPose;                  // текущее фактическое положение маркера относительно ГСК
geometry_msgs::PoseStamped estimateCurrentArucoOdomPose;          // текущее   оценочное положение маркера относительно ГСК

bool init = false;
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

    listener.transformPose("camera_frame", currentArucoOdomPose, currentArucoCameraPose);
    
    // listener.lookupTransform("aruco_frame", "camera_frame", ros::Time(0), transform);
    // currentArucoCameraPose.header.frame_id = "aruco_frame";
    // currentArucoCameraPose.pose.position.x = transform.getOrigin().x();
    // currentArucoCameraPose.pose.position.y = transform.getOrigin().y();
    // currentArucoCameraPose.pose.position.z = transform.getOrigin().z();
    // currentArucoCameraPose.pose.orientation.w = transform.getRotation().getW();
    // currentArucoCameraPose.pose.orientation.x = transform.getRotation().getX();
    // currentArucoCameraPose.pose.orientation.y = transform.getRotation().getY();
    // currentArucoCameraPose.pose.orientation.z = transform.getRotation().getZ();

    std::cout << std::endl;
    ROS_INFO("\n"
              "<--------------odom:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->\n"
              "<------camera_frame:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->",
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

// получаем текущее оценочное положение маркера относительно ГСК
void getEstimeteCurrentArucoCameraPose(){
  if (init) estimateCurrentArucoOdomPose = currentArucoOdomPose;
  init = false;
}

// расстояние между точками
double getDistance(const double t1, const double t2) {                
  return t1 - t2;
}

void setres2Write(){
  res2Write.estimateArCamPose   = estimateCurrentArucoCameraPose;     // текущее оценочное   положение маркера относительно камеры
  res2Write.trueArCamPose       = currentArucoCameraPose;             // текущее фактическое положение маркера относительно камеры
  res2Write.estimateArOdomPose  = estimateCurrentArucoOdomPose;       // текущее   оценочное положение маркера относительно ГСК
  res2Write.trueArOdomPose      = currentArucoOdomPose;               // текущее фактическое положение маркера относительно ГСК
}

bool factEqualEstimate(){
  if (estimateCurrentArucoCameraPose.pose.position.x == currentArucoCameraPose.pose.position.x) return true;
  if (estimateCurrentArucoCameraPose.pose.position.y == currentArucoCameraPose.pose.position.z) return true;
  if (estimateCurrentArucoCameraPose.pose.position.z == currentArucoCameraPose.pose.position.z) return true;
  return false;
}

// записываем результаты эксперимента
void writeArucoPoseData2Bag(){                                        

  if (currentArucoCameraPose == prevArucoCameraPose) return;
  if (factEqualEstimate()) return; 

  geometry_msgs::Vector3 markerOffset; // фактическое смещение маркера
  
  // фактическое перемещение маркера
  markerOffset.x = abs(getDistance(currentArucoCameraPose.pose.position.x, prevArucoCameraPose.pose.position.x));
  markerOffset.y = abs(getDistance(currentArucoCameraPose.pose.position.y, prevArucoCameraPose.pose.position.y));
  markerOffset.z = abs(getDistance(currentArucoCameraPose.pose.position.z, prevArucoCameraPose.pose.position.z));
  double markerSpaceOffset = std::sqrt(std::pow(markerOffset.x, 2) + std::pow(markerOffset.y, 2) + std::pow(markerOffset.z, 2));
  
  if (markerSpaceOffset > MEASURE_STEP) {
    prevArucoCameraPose = currentArucoCameraPose;
    setres2Write();
    arucoResDataBagPub.publish(res2Write);
    ROS_INFO("\n"
              "######################################################\n"
              "[markerSpaceOffset = %.5f] \n"
              "!!! SEND POSES DATA !!!\n"
              "estimateCurrentArucoCameraPose.pose.position.x = %.5f \n"
              "estimateCurrentArucoCameraPose.pose.position.y = %.5f \n"
              "estimateCurrentArucoCameraPose.pose.position.z = %.5f \n"
              "------------------------------------------------------\n"
              "currentArucoCameraPose.pose.position.x = %.5f \n"
              "currentArucoCameraPose.pose.position.y = %.5f \n"
              "currentArucoCameraPose.pose.position.z = %.5f \n"
              "######################################################\n\n",
              
              markerSpaceOffset,

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

  ros::Subscriber arucoCameraPoseSub =                                             // текущее оценочное положение маркера относительно камеры
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);

  ros::Subscriber arucoGazeboPoseSub =                                             // текущее фактическое положение маркера относительно ГСК
    node.subscribe("/gazebo/model_states", 0, getCurrentArucoOdomPose);

  arucoResDataBagPub =
    node.advertise<send_results_vdrk::Poses4>("arucoResDataBagTopic", 0);

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    ros::spinOnce();
    
    allCallbacksCall = getEstimateCurrentArucoCameraPoseCallback &&
                          getCurrentArucoOdomPoseCallback;

    if (!allCallbacksCall) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;
    getCurrentArucoOdomPoseCallback           = false;

    getCurrentArucoCameraPose(listener);                                           // текущее фактическое положение маркера относительно камеры
    getEstimeteCurrentArucoCameraPose();                                           // текущее   оценочное положение маркера относительно ГСК
    
    writeArucoPoseData2Bag();

    loop_rate.sleep();
  }

  return 0;
}