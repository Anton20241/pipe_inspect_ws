#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include "send_results_vdrk/Poses4.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

#define MEASURE_STEP 0.05                             // [м] = 5 см - шаг измерения положения маркера

// сообщение в топик на запись данных
send_results_vdrk::Poses4 res2WriteAr;
send_results_vdrk::Poses4 res2WriteCam;

// паблишер в топик на запись данных
ros::Publisher arucoResDataPub;
ros::Publisher cameraResDataPub;

geometry_msgs::Twist velMarker;                       // текущая скорость _маркера_
geometry_msgs::Twist velCamera;                       // текущая скорость _камеры_

geometry_msgs::PoseStamped estCrntArZrPntPose;        // текущее      оценочное положение _маркера_ относительно 0 точки
geometry_msgs::PoseStamped estPrevArZrPntPose;        // предыдущее   оценочное положение _маркера_ относительно 0 точки
geometry_msgs::PoseStamped crntArZrPntPose;           // текущее    фактическое положение _маркера_ относительно 0 точки
geometry_msgs::PoseStamped crntArOdomPose;            // текущее    фактическое положение _маркера_ относительно ГСК
geometry_msgs::PoseStamped prevArOdomPose;            // предыдущее фактическое положение _маркера_ относительно ГСК
geometry_msgs::PoseStamped estCrntArOdomPose;         // текущее      оценочное положение _маркера_ относительно ГСК

geometry_msgs::PoseStamped estCrntCamZrPntPose;       // текущее      оценочное положение _камеры_ относительно 0 точки
geometry_msgs::PoseStamped estPrevCamZrPntPose;       // предыдущее   оценочное положение _камеры_ относительно 0 точки
geometry_msgs::PoseStamped crntCamZrPntPose;          // текущее    фактическое положение _камеры_ относительно 0 точки
geometry_msgs::PoseStamped crntCamOdomPose;           // текущее    фактическое положение _камеры_ относительно ГСК
geometry_msgs::PoseStamped prevCamOdomPose;           // предыдущее фактическое положение _камеры_ относительно ГСК
geometry_msgs::PoseStamped estCrntCamOdomPose;        // текущее      оценочное положение _камеры_ относительно ГСК

geometry_msgs::PoseStamped estCrntArCamPoseState;     // конечная точка _маркера_ до начала его перемещения

bool allPosesGet        = false;                      // все положения получены
bool getPoseEstZrPnt    = false;
bool getPoseFactZrPnt   = false;
bool getPoseEstOdom     = false;
bool getPoseFactOdom    = false;

bool start = true;

// получаем текущую скорость _маркера_
void getVelMarker(const geometry_msgs::Twist& velMarkerMsg){
  velMarker = velMarkerMsg;
}

// получаем текущую скорость _камеры_
void getVelCamera(const geometry_msgs::Twist& velCameraMsg){
  velCamera = velCameraMsg;
}

bool isStop(const geometry_msgs::Twist& vel){
  if (vel.angular.x != 0) return false;
  if (vel.angular.y != 0) return false;
  if (vel.angular.z != 0) return false;
  if (vel.linear.x  != 0) return false;
  if (vel.linear.y  != 0) return false;
  if (vel.linear.z  != 0) return false;

  return true;
}

// получаем текущее оценочное положение _маркера_ относительно 0 точки И
// получаем текущее оценочное положение _камеры_  относительно 0 точки
void getCrntEstZrPntPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 

  estPrevArZrPntPose    = estCrntArZrPntPose;
  estPrevCamZrPntPose   = estCrntCamZrPntPose;

  estCrntArZrPntPose    = arucoCameraMsg;

  if (isStop(velCamera)) estCrntArCamPoseState = estCrntArZrPntPose;

  estCrntCamZrPntPose.pose.position.x = estCrntArCamPoseState.pose.position.x - estCrntArZrPntPose.pose.position.x;
  estCrntCamZrPntPose.pose.position.y = estCrntArCamPoseState.pose.position.y - estCrntArZrPntPose.pose.position.y + 0.15;
  estCrntCamZrPntPose.pose.position.z = estCrntArCamPoseState.pose.position.z - estCrntArZrPntPose.pose.position.z;

  getPoseEstZrPnt = true;
}

// получаем текущее фактическое положение _маркера_ относительно ГСК И
// получаем текущее фактическое положение _камеры_  относительно ГСК 
void getCrntFactOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) {

  prevArOdomPose  = crntArOdomPose;
  prevCamOdomPose = crntCamOdomPose;

  // маркер
  crntArOdomPose.pose.position.x     = arucoGazeboMsg.pose[1].position.x;
  crntArOdomPose.pose.position.y     = arucoGazeboMsg.pose[1].position.y;
  crntArOdomPose.pose.position.z     = arucoGazeboMsg.pose[1].position.z;
  crntArOdomPose.pose.orientation.w  = arucoGazeboMsg.pose[1].orientation.w;
  crntArOdomPose.pose.orientation.x  = arucoGazeboMsg.pose[1].orientation.x;
  crntArOdomPose.pose.orientation.y  = arucoGazeboMsg.pose[1].orientation.y;
  crntArOdomPose.pose.orientation.z  = arucoGazeboMsg.pose[1].orientation.z;
  // камера
  crntCamOdomPose.pose.position.x    = arucoGazeboMsg.pose[2].position.x;
  crntCamOdomPose.pose.position.y    = arucoGazeboMsg.pose[2].position.y;
  crntCamOdomPose.pose.position.z    = arucoGazeboMsg.pose[2].position.z;
  crntCamOdomPose.pose.orientation.w = arucoGazeboMsg.pose[2].orientation.w;
  crntCamOdomPose.pose.orientation.x = arucoGazeboMsg.pose[2].orientation.x;
  crntCamOdomPose.pose.orientation.y = arucoGazeboMsg.pose[2].orientation.y;
  crntCamOdomPose.pose.orientation.z = arucoGazeboMsg.pose[2].orientation.z;

  getPoseFactOdom = true;
}

// получаем текущее фактическое положение _маркера_ относительно 0 точки И
// получаем текущее фактическое положение _камеры_  относительно 0 точки
void getCrntFactZrPntPose(const tf::TransformListener& listener){

  try{
    crntArOdomPose.header.frame_id  = "odom";
    crntCamOdomPose.header.frame_id = "odom";
    crntArOdomPose.header.stamp     = ros::Time();
    crntCamOdomPose.header.stamp    = ros::Time();

    listener.transformPose("camera_link_optical", crntArOdomPose,   crntArZrPntPose);
    listener.transformPose("camera_link_optical", crntCamOdomPose,  crntCamZrPntPose);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  getPoseFactZrPnt = true;
}

// оценочное перемещение _маркера_ относительно ГСК
void getEstCrntArOdomPose(const tf::TransformListener& listener){

  if (!isStop(velCamera)){
    return;
  }

  geometry_msgs::PoseStamped delta_XYZ_estimateZrPnt;                     // оценочное перемещение _маркера_ относительно 0 точки
  geometry_msgs::PoseStamped delta_XYZ_estimateOdom;                      // оценочное перемещение _маркера_ относительно ГСК

  try{
    delta_XYZ_estimateZrPnt.header.frame_id    = "camera_link_optical";
    delta_XYZ_estimateZrPnt.header.stamp       = ros::Time();
    delta_XYZ_estimateZrPnt.pose.orientation.x = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.y = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.z = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.w = 1;
    delta_XYZ_estimateZrPnt.pose.position.x    = estCrntArZrPntPose.pose.position.x - estPrevArZrPntPose.pose.position.x;
    delta_XYZ_estimateZrPnt.pose.position.y    = estCrntArZrPntPose.pose.position.y - estPrevArZrPntPose.pose.position.y;
    delta_XYZ_estimateZrPnt.pose.position.z    = estCrntArZrPntPose.pose.position.z - estPrevArZrPntPose.pose.position.z;

    delta_XYZ_estimateOdom                   = delta_XYZ_estimateZrPnt;
    delta_XYZ_estimateOdom.pose.position.x   = ( 1) * delta_XYZ_estimateZrPnt.pose.position.x;
    delta_XYZ_estimateOdom.pose.position.y   = ( 1) * delta_XYZ_estimateZrPnt.pose.position.z;
    delta_XYZ_estimateOdom.pose.position.z   = (-1) * delta_XYZ_estimateZrPnt.pose.position.y;

    // listener.transformPose("odom", delta_XYZ_estimateZrPnt, delta_XYZ_estimateOdom);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  estCrntArOdomPose.pose.position.x += delta_XYZ_estimateOdom.pose.position.x;
  estCrntArOdomPose.pose.position.y += delta_XYZ_estimateOdom.pose.position.y;
  estCrntArOdomPose.pose.position.z += delta_XYZ_estimateOdom.pose.position.z;

}

// оценочное перемещение _камеры_ относительно ГСК
void getEstCrntCamOdomPose(const tf::TransformListener& listener){

  if (isStop(velCamera)){
    return;
  }
  
  geometry_msgs::PoseStamped delta_XYZ_estimateZrPnt;                     // оценочное перемещение _камеры_ относительно 0 точки
  geometry_msgs::PoseStamped delta_XYZ_estimateOdom;                      // оценочное перемещение _камеры_ относительно ГСК
  
  try{
    delta_XYZ_estimateZrPnt.header.frame_id    = "camera_link_optical";
    delta_XYZ_estimateZrPnt.header.stamp       = ros::Time();
    delta_XYZ_estimateZrPnt.pose.orientation.x = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.y = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.z = 0;
    delta_XYZ_estimateZrPnt.pose.orientation.w = 1;
    delta_XYZ_estimateZrPnt.pose.position.x    = estCrntCamZrPntPose.pose.position.x - estPrevCamZrPntPose.pose.position.x;
    delta_XYZ_estimateZrPnt.pose.position.y    = estCrntCamZrPntPose.pose.position.y - estPrevCamZrPntPose.pose.position.y;
    delta_XYZ_estimateZrPnt.pose.position.z    = estCrntCamZrPntPose.pose.position.z - estPrevCamZrPntPose.pose.position.z;

    delta_XYZ_estimateOdom                   = delta_XYZ_estimateZrPnt;
    delta_XYZ_estimateOdom.pose.position.x   = ( 1) * delta_XYZ_estimateZrPnt.pose.position.x;
    delta_XYZ_estimateOdom.pose.position.y   = ( 1) * delta_XYZ_estimateZrPnt.pose.position.z;
    delta_XYZ_estimateOdom.pose.position.z   = (-1) * delta_XYZ_estimateZrPnt.pose.position.y;

    // listener.transformPose("odom", delta_XYZ_estimateZrPnt, delta_XYZ_estimateOdom);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  estCrntCamOdomPose.pose.position.x += delta_XYZ_estimateOdom.pose.position.x;
  estCrntCamOdomPose.pose.position.y += delta_XYZ_estimateOdom.pose.position.y;
  estCrntCamOdomPose.pose.position.z += delta_XYZ_estimateOdom.pose.position.z;

}

// получаем текущее оценочное положение _маркера_ относительно ГСК И
// получаем текущее оценочное положение _камеры_  относительно ГСК
void getCrntEstOdomPose(const tf::TransformListener& listener){

  if (start){
    start = false;
    estCrntArOdomPose   = crntArOdomPose;        // текущее оц пол-е _маркера_ отн ГСК = текущее факт пол-е _маркера_ отн ГСК
    estCrntCamOdomPose  = crntCamOdomPose;       // текущее оц пол-е _камеры_  отн ГСК = текущее факт пол-е _камеры_  отн ГСК
    // estCrntArOdomPose.pose.orientation.w = 1;
    // estCrntArOdomPose.pose.orientation.x = 0;
    // estCrntArOdomPose.pose.orientation.y = 0;
    // estCrntArOdomPose.pose.orientation.z = 0;
    // estCrntArOdomPose.pose.position.x    = 0;
    // estCrntArOdomPose.pose.position.y    = 0;
    // estCrntArOdomPose.pose.position.z    = 0;
    // estCrntCamOdomPose = estCrntArOdomPose;
    getPoseEstOdom     = true;
    return;
  }

  getEstCrntArOdomPose(listener);
  getEstCrntCamOdomPose(listener);

  getPoseEstOdom = true;

}

void showPoses(){

  if      (!isStop(velMarker)) printf("\n-------------------------ЕДЕТ МАРКЕР-------------------\n");
  else if (!isStop(velCamera)) printf("\n-------------------------ЕДЕТ КАМЕРА-------------------\n");
  else                         printf("\n--------------------------ВСЕ СТОИТ--------------------\n");

  printf("######################################################\n");
  printf("[текущее оценочное положение _маркера_ относительно 0 точки]\n");
  printf("estCrntArZrPntPose.pose.position.x  = %f\n", estCrntArZrPntPose.pose.position.x);
  printf("estCrntArZrPntPose.pose.position.y  = %f\n", estCrntArZrPntPose.pose.position.y);
  printf("estCrntArZrPntPose.pose.position.z  = %f\n", estCrntArZrPntPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно 0 точки]\n");
  printf("crntArZrPntPose.pose.position.x     = %f\n", crntArZrPntPose.pose.position.x);
  printf("crntArZrPntPose.pose.position.y     = %f\n", crntArZrPntPose.pose.position.y);
  printf("crntArZrPntPose.pose.position.z     = %f\n", crntArZrPntPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее оценочное положение _маркера_ относительно ГСК]\n");
  printf("estCrntArOdomPose.pose.position.x   = %f\n", estCrntArOdomPose.pose.position.x);
  printf("estCrntArOdomPose.pose.position.y   = %f\n", estCrntArOdomPose.pose.position.y);
  printf("estCrntArOdomPose.pose.position.z   = %f\n", estCrntArOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно ГСК]\n");
  printf("crntArOdomPose.pose.position.x      = %f\n", crntArOdomPose.pose.position.x);
  printf("crntArOdomPose.pose.position.y      = %f\n", crntArOdomPose.pose.position.y);
  printf("crntArOdomPose.pose.position.z      = %f\n", crntArOdomPose.pose.position.z);
  printf("\n######################################################\n");
  printf("[текущее оценочное положение _камеры_ относительно 0 точки]\n");
  printf("estCrntCamZrPntPose.pose.position.x = %f\n", estCrntCamZrPntPose.pose.position.x);
  printf("estCrntCamZrPntPose.pose.position.y = %f\n", estCrntCamZrPntPose.pose.position.y);
  printf("estCrntCamZrPntPose.pose.position.z = %f\n", estCrntCamZrPntPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно 0 точки]\n");
  printf("crntCamZrPntPose.pose.position.x    = %f\n", crntCamZrPntPose.pose.position.x);
  printf("crntCamZrPntPose.pose.position.y    = %f\n", crntCamZrPntPose.pose.position.y);
  printf("crntCamZrPntPose.pose.position.z    = %f\n", crntCamZrPntPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее оценочное положение _камеры_ относительно ГСК]\n");
  printf("estCrntCamOdomPose.pose.position.x  = %f\n", estCrntCamOdomPose.pose.position.x);
  printf("estCrntCamOdomPose.pose.position.y  = %f\n", estCrntCamOdomPose.pose.position.y);
  printf("estCrntCamOdomPose.pose.position.z  = %f\n", estCrntCamOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно ГСК]\n");
  printf("crntCamOdomPose.pose.position.x     = %f\n", crntCamOdomPose.pose.position.x);
  printf("crntCamOdomPose.pose.position.y     = %f\n", crntCamOdomPose.pose.position.y);
  printf("crntCamOdomPose.pose.position.z     = %f\n", crntCamOdomPose.pose.position.z);
  printf("######################################################\n");
}

// расстояние между точками
double getDistance(const double t1, const double t2) {                
  return t1 - t2;
}

void setRes2Write(){
  res2WriteAr.estimateZeroPointPose   = estCrntArZrPntPose;     // текущее оценочное   положение _маркера_ относительно 0 точки
  res2WriteAr.trueZeroPointPose       = crntArZrPntPose;        // текущее фактическое положение _маркера_ относительно 0 точки
  res2WriteAr.estimateOdomPose        = estCrntArOdomPose;      // текущее   оценочное положение _маркера_ относительно ГСК
  res2WriteAr.trueOdomPose            = crntArOdomPose;         // текущее фактическое положение _маркера_ относительно ГСК

  res2WriteCam.estimateZeroPointPose  = estCrntCamZrPntPose;    // текущее оценочное   положение _камеры_ относительно 0 точки
  res2WriteCam.trueZeroPointPose      = crntCamZrPntPose;       // текущее фактическое положение _камеры_ относительно 0 точки
  res2WriteCam.estimateOdomPose       = estCrntCamOdomPose;     // текущее   оценочное положение _камеры_ относительно ГСК
  res2WriteCam.trueOdomPose           = crntCamOdomPose;        // текущее фактическое положение _камеры_ относительно ГСК
}

// записываем результаты эксперимента
void writeArucoPoseData2Bag(){

  static bool startWrite = true;
  static geometry_msgs::PoseStamped crntArOdomPoseOld;          // фактическое положение _маркера_ относительно ГСК старое
  static geometry_msgs::PoseStamped crntCamOdomPoseOld;         // фактическое положение _камеры_  относительно ГСК старое

  if (startWrite){
    crntArOdomPoseOld   = crntArOdomPose;
    crntCamOdomPoseOld  = crntCamOdomPose;
    startWrite          = false;
  }

  // если _камера_ стоит, записываем результаты _маркера_
  if (isStop(velCamera)){
    geometry_msgs::Vector3 markerOffset;                        // фактическое перемещение _маркера_ относительно ГСК
    markerOffset.x = abs(getDistance(crntArOdomPoseOld.pose.position.x, crntArOdomPose.pose.position.x));
    markerOffset.y = abs(getDistance(crntArOdomPoseOld.pose.position.y, crntArOdomPose.pose.position.y));
    markerOffset.z = abs(getDistance(crntArOdomPoseOld.pose.position.z, crntArOdomPose.pose.position.z));
    double markerSpaceOffset = std::sqrt(std::pow(markerOffset.x, 2) + std::pow(markerOffset.y, 2) + std::pow(markerOffset.z, 2));

    if (markerSpaceOffset > MEASURE_STEP) {
      crntArOdomPoseOld = crntArOdomPose;
      setRes2Write();
      arucoResDataPub.publish(res2WriteAr);
      showPoses();
    }
  }

  // если _маркер_ стоит, записываем результаты _камеры_
  if (isStop(velMarker)){
    geometry_msgs::Vector3 cameraOffset;                        // фактическое перемещение _камеры_ относительно ГСК
    cameraOffset.x = abs(getDistance(crntCamOdomPoseOld.pose.position.x, crntCamOdomPose.pose.position.x));
    cameraOffset.y = abs(getDistance(crntCamOdomPoseOld.pose.position.y, crntCamOdomPose.pose.position.y));
    cameraOffset.z = abs(getDistance(crntCamOdomPoseOld.pose.position.z, crntCamOdomPose.pose.position.z));
    double cameraSpaceOffset = std::sqrt(std::pow(cameraOffset.x, 2) + std::pow(cameraOffset.y, 2) + std::pow(cameraOffset.z, 2));

    if (cameraSpaceOffset > MEASURE_STEP) {
      crntCamOdomPoseOld = crntCamOdomPose;
      setRes2Write();
      cameraResDataPub.publish(res2WriteCam);
      showPoses();
    }
  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "send_res_vdrk");
  ros::NodeHandle node;
  tf::TransformListener listener;

  // текущее оценочное положение относительно 0 точки
  ros::Subscriber crntEstZrPntPoseSub =                                             
    node.subscribe("/aruco_single/pose", 0, getCrntEstZrPntPose);

  // текущее фактическое положение относительно ГСК
  ros::Subscriber CrntFactOdomPoseSub =                                             
    node.subscribe("/gazebo/model_states", 0, getCrntFactOdomPose);

  // текущая скорость _маркера_
  ros::Subscriber velMarkerSub =                                             
    node.subscribe("/aruco_cmd_vel", 0, getVelMarker);

  // текущая скорость _камеры_
  ros::Subscriber velCameraSub =                                             
    node.subscribe("/camera_cmd_vel", 0, getVelCamera);

  arucoResDataPub =
    node.advertise<send_results_vdrk::Poses4>("arucoPosesTopic", 0);

  cameraResDataPub =
    node.advertise<send_results_vdrk::Poses4>("cameraPosesTopic", 0);

  ros::Rate loop_rate(30);

  while (ros::ok()){

    ros::spinOnce();

    if (!getPoseEstZrPnt) continue;
    if (!getPoseFactOdom) continue;

    
    // текущее фактическое положение относительно 0 точки
    getCrntFactZrPntPose(listener);

    // текущее оценочное положение относительно ГСК
    getCrntEstOdomPose(listener);

    allPosesGet = getPoseEstZrPnt && getPoseFactZrPnt && getPoseEstOdom && getPoseFactOdom;

    if (!allPosesGet) continue;

    getPoseEstZrPnt     = false;
    getPoseFactZrPnt    = false;
    getPoseEstOdom      = false;
    getPoseFactOdom     = false;

    writeArucoPoseData2Bag();
    
    loop_rate.sleep();
  }

  return 0;
}