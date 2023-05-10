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
geometry_msgs::PoseStamped estimatePrevArucoCameraPose;           // предыдущее   оценочное положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoCameraPose;                // текущее    фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped prevArucoCameraPose;                   // предыдущее фактическое положение маркера относительно камеры
geometry_msgs::PoseStamped currentArucoOdomPose;                  // текущее    фактическое положение маркера относительно ГСК
geometry_msgs::PoseStamped prevArucoOdomPose;                     // предыдущее фактическое положение маркера относительно ГСК
geometry_msgs::PoseStamped estimateCurrentArucoOdomPose;          // текущее      оценочное положение маркера относительно ГСК

bool allPosesGet      = false;                                    // все положения получены
bool getPoseEstCam    = false;
bool getPoseFactCam   = false;
bool getPoseEstOdom   = false;
bool getPoseFactOdom  = false;

bool start = true;

// получаем текущее оценочное положение маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getPoseEstCam = true;
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

  getPoseFactOdom = true;
}

// получаем текущее фактическое положение маркера относительно камеры
void getCurrentArucoCameraPose(const tf::TransformListener& listener){
  if(!getPoseFactOdom) return;
  if(!getPoseEstCam)   return;

  try{
    currentArucoOdomPose.header.frame_id = "odom";
    currentArucoOdomPose.header.stamp    = ros::Time();

    listener.transformPose("camera_link_optical", currentArucoOdomPose, currentArucoCameraPose);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  getPoseFactCam = true;
}

bool equal(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2, double precision){
  if (abs(pose1.pose.position.x - pose2.pose.position.x) > precision) return false;
  if (abs(pose1.pose.position.y - pose2.pose.position.y) > precision) return false;
  if (abs(pose1.pose.position.z - pose2.pose.position.z) > precision) return false;
  return true;
}

// получаем текущее оценочное положение маркера относительно ГСК
void getEstimateCurrentArucoOdomPose(const tf::TransformListener& listener){
  if(!getPoseFactOdom) return;
  if(!getPoseEstCam)   return;

  if (start){
    start = false;
    estimateCurrentArucoOdomPose = currentArucoOdomPose;          //текущее оц пол-е маркера отн ГСК = текущее факт пол-е маркера отн ГСК
    estimatePrevArucoCameraPose = estimateCurrentArucoCameraPose; //пред оц пол-е маркера отн камеры = текущее оц пол-е маркера отн камеры
    getPoseEstOdom = true;
    return;
  }

  if (equal(currentArucoOdomPose, prevArucoOdomPose, 0.001)){           // передний маркер относительно мира   не двигался
    estimatePrevArucoCameraPose = estimateCurrentArucoCameraPose;
    getPoseEstOdom = true;
    return;
  };    

  geometry_msgs::PoseStamped delta_XYZ_estimateCam;                     // оценочное перемещение маркера относительно камеры
  geometry_msgs::PoseStamped delta_XYZ_estimateOdom;                    // оценочное перемещение маркера относительно камеры

  try{
    delta_XYZ_estimateCam.header.frame_id    = "camera_link_optical";
    delta_XYZ_estimateCam.header.stamp       = ros::Time();
    delta_XYZ_estimateCam.pose.orientation.x = 0;
    delta_XYZ_estimateCam.pose.orientation.y = 0;
    delta_XYZ_estimateCam.pose.orientation.z = 0;
    delta_XYZ_estimateCam.pose.orientation.w = 1;
    delta_XYZ_estimateCam.pose.position.x    = estimateCurrentArucoCameraPose.pose.position.x - estimatePrevArucoCameraPose.pose.position.x;
    delta_XYZ_estimateCam.pose.position.y    = estimateCurrentArucoCameraPose.pose.position.y - estimatePrevArucoCameraPose.pose.position.y;
    delta_XYZ_estimateCam.pose.position.z    = estimateCurrentArucoCameraPose.pose.position.z - estimatePrevArucoCameraPose.pose.position.z;
    estimatePrevArucoCameraPose              = estimateCurrentArucoCameraPose;
    delta_XYZ_estimateOdom                   = delta_XYZ_estimateCam;

    delta_XYZ_estimateOdom.pose.position.x   = ( 1) * delta_XYZ_estimateCam.pose.position.x;
    delta_XYZ_estimateOdom.pose.position.y   = ( 1) * delta_XYZ_estimateCam.pose.position.z;
    delta_XYZ_estimateOdom.pose.position.z   = (-1) * delta_XYZ_estimateCam.pose.position.y;

    //listener.transformPose("odom", delta_XYZ_estimateCam, delta_XYZ_estimateOdom);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  estimateCurrentArucoOdomPose.pose.position.x += delta_XYZ_estimateOdom.pose.position.x;
  estimateCurrentArucoOdomPose.pose.position.y += delta_XYZ_estimateOdom.pose.position.y;
  estimateCurrentArucoOdomPose.pose.position.z += delta_XYZ_estimateOdom.pose.position.z;

  getPoseEstOdom = true;

}

void showPoses(){
  printf("\n######################################################\n");
  printf("[текущее оценочное положение маркера относительно камеры]\n");
  printf("estimateCurrentArucoCameraPose.pose.position.x = %f\n", estimateCurrentArucoCameraPose.pose.position.x);
  printf("estimateCurrentArucoCameraPose.pose.position.y = %f\n", estimateCurrentArucoCameraPose.pose.position.y);
  printf("estimateCurrentArucoCameraPose.pose.position.z = %f\n", estimateCurrentArucoCameraPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение маркера относительно камеры]\n");
  printf("currentArucoCameraPose.pose.position.x         = %f\n", currentArucoCameraPose.pose.position.x);
  printf("currentArucoCameraPose.pose.position.y         = %f\n", currentArucoCameraPose.pose.position.y);
  printf("currentArucoCameraPose.pose.position.z         = %f\n", currentArucoCameraPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее оценочное положение маркера относительно ГСК]\n");
  printf("estimateCurrentArucoOdomPose.pose.position.x   = %f\n", estimateCurrentArucoOdomPose.pose.position.x);
  printf("estimateCurrentArucoOdomPose.pose.position.y   = %f\n", estimateCurrentArucoOdomPose.pose.position.y);
  printf("estimateCurrentArucoOdomPose.pose.position.z   = %f\n", estimateCurrentArucoOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение маркера относительно ГСК]\n");
  printf("currentArucoOdomPose.pose.position.x           = %f\n", currentArucoOdomPose.pose.position.x);
  printf("currentArucoOdomPose.pose.position.y           = %f\n", currentArucoOdomPose.pose.position.y);
  printf("currentArucoOdomPose.pose.position.z           = %f\n", currentArucoOdomPose.pose.position.z);
  printf("######################################################\n");
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

// записываем результаты эксперимента
void writeArucoPoseData2Bag(){
  //showPoses();

  if (equal(currentArucoOdomPose, prevArucoOdomPose, 0.001)){         // передний маркер относительно мира   не двигался
    return;
  };

  geometry_msgs::Vector3 markerOffset; // фактическое перемещение маркера относительно ГСК
  markerOffset.x    = abs(getDistance(currentArucoOdomPose.pose.position.x, prevArucoOdomPose.pose.position.x));
  markerOffset.y    = abs(getDistance(currentArucoOdomPose.pose.position.y, prevArucoOdomPose.pose.position.y));
  markerOffset.z    = abs(getDistance(currentArucoOdomPose.pose.position.z, prevArucoOdomPose.pose.position.z));
  double markerSpaceOffset = std::sqrt(std::pow(markerOffset.x, 2) + std::pow(markerOffset.y, 2) + std::pow(markerOffset.z, 2));

  if (markerSpaceOffset > MEASURE_STEP) {
    prevArucoOdomPose = currentArucoOdomPose;
    setres2Write();
    arucoResDataBagPub.publish(res2Write);
    showPoses();
  } else {
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
    getCurrentArucoCameraPose(listener);                            // текущее фактическое положение маркера относительно камеры
    getEstimateCurrentArucoOdomPose(listener);                      // текущее   оценочное положение маркера относительно ГСК

    allPosesGet = getPoseEstCam && getPoseFactCam && getPoseEstOdom && getPoseFactOdom;

    if (!allPosesGet) continue;

    getPoseEstCam     = false;
    getPoseFactCam    = false;
    getPoseEstOdom    = false;
    getPoseFactOdom   = false;

    writeArucoPoseData2Bag();
    
    loop_rate.sleep();
  }

  return 0;
}