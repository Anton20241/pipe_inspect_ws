#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include "send_results_vdrk/VdrkPose.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <stdlib.h>

#define MEASURE_STEP 0.05                             // [м] = 5 см - шаг дискретизации (измерения)

// сообщения в топик на запись полученных положений
send_results_vdrk::VdrkPose vdrkPose;

// паблишер в топик на запись данных
ros::Publisher vdrkPosePub;

// подписчики на текущие положения
ros::Subscriber estCrntPoseSub;
ros::Subscriber crntOdomPoseSub;
ros::Subscriber velCrntArSub;
ros::Subscriber velCrntCamSub;

geometry_msgs::Twist velCrntAr;                       // текущая скорость _маркера_
geometry_msgs::Twist velCrntCam;                      // текущая скорость _камеры_

geometry_msgs::PoseStamped estCrntArCamPose;          // текущее    оценочное   положение _маркера_ относительно _камеры_
geometry_msgs::PoseStamped estPrevArCamPose;          // предыдущее оценочное   положение _маркера_ относительно _камеры_
geometry_msgs::PoseStamped crntArCamPose;             // текущее    фактическое положение _маркера_ относительно _камеры_
geometry_msgs::PoseStamped crntArOdomPose;            // текущее    фактическое положение _маркера_ относительно ГСК     
geometry_msgs::PoseStamped estCrntArOdomPose;         // текущее    оценочное   положение _маркера_ относительно ГСК     
geometry_msgs::PoseStamped crntCamOdomPose;           // текущее    фактическое положение _камеры_  относительно ГСК     
geometry_msgs::PoseStamped estCrntCamOdomPose;        // текущее    оценочное   положение _камеры_  относительно ГСК     

bool allPosesGet            = false;                  // все положения получены
bool getEstCrntArCamPose    = false;                  // получено оценочное положение _маркера_ относительно _камеры_
bool getOdomPoses           = false;                  // получены фактические положения относительно ГСК
bool initVdrkPoseIsSet      = false;                  // заданы начальные условия (положения) ВРДК

// получаем текущую скорость _маркера_
void getVelCrntArHandler(const geometry_msgs::Twist& velCrntArMsg){
  velCrntAr = velCrntArMsg;
}

// получаем текущую скорость _камеры_
void getVelCrntCamHandler(const geometry_msgs::Twist& velCrntCamMsg){
  velCrntCam = velCrntCamMsg;
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

// получаем текущее оценочное положение _маркера_ относительно _камеры_
void getEstArCamPoseHandler(const geometry_msgs::PoseStamped& estCrntArCamPoseMsg) {
  estCrntArCamPose    = estCrntArCamPoseMsg;
  getEstCrntArCamPose = true;
}

// получаем текущее фактическое положение _маркера_ относительно ГСК И
// получаем текущее фактическое положение _камеры_  относительно ГСК 
void getCrntOdomPoseHandler(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  crntArOdomPose.pose.position.x     = arucoGazeboMsg.pose[1].position.x;
  crntArOdomPose.pose.position.y     = arucoGazeboMsg.pose[1].position.y;
  crntArOdomPose.pose.position.z     = arucoGazeboMsg.pose[1].position.z;
  crntArOdomPose.pose.orientation.w  = arucoGazeboMsg.pose[1].orientation.w;
  crntArOdomPose.pose.orientation.x  = arucoGazeboMsg.pose[1].orientation.x;
  crntArOdomPose.pose.orientation.y  = arucoGazeboMsg.pose[1].orientation.y;
  crntArOdomPose.pose.orientation.z  = arucoGazeboMsg.pose[1].orientation.z;
  crntCamOdomPose.pose.position.x    = arucoGazeboMsg.pose[2].position.x;
  crntCamOdomPose.pose.position.y    = arucoGazeboMsg.pose[2].position.y;
  crntCamOdomPose.pose.position.z    = arucoGazeboMsg.pose[2].position.z;
  crntCamOdomPose.pose.orientation.w = arucoGazeboMsg.pose[2].orientation.w;
  crntCamOdomPose.pose.orientation.x = arucoGazeboMsg.pose[2].orientation.x;
  crntCamOdomPose.pose.orientation.y = arucoGazeboMsg.pose[2].orientation.y;
  crntCamOdomPose.pose.orientation.z = arucoGazeboMsg.pose[2].orientation.z;
  getOdomPoses = true;
}

// получаем текущее фактическое положение _маркера_ относительно _камеры_
void getCrntArCamPose(const tf::TransformListener& listener){
  try{
    crntArOdomPose.header.frame_id  = "odom";
    crntArOdomPose.header.stamp     = ros::Time();
    listener.transformPose("camera_link_optical", crntArOdomPose, crntArCamPose);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void sendOdom2EstCamLinkBaseTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( estCrntCamOdomPose.pose.position.x, 
                                    estCrntCamOdomPose.pose.position.y, 
                                    estCrntCamOdomPose.pose.position.z) );
  transform.setRotation( tf::Quaternion(estCrntCamOdomPose.pose.orientation.x, 
                                        estCrntCamOdomPose.pose.orientation.y,
                                        estCrntCamOdomPose.pose.orientation.z,
                                        estCrntCamOdomPose.pose.orientation.w) );
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "odom", "est_camera_link_base_direct") );
  ROS_INFO("odom -> est_camera_link_base_direct");
}

void sendEstCamLinkBase2EstCamLinkOptTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476) );
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "est_camera_link_base_direct", "est_camera_link_optical_direct") );
  ROS_INFO("est_camera_link_base_direct -> est_camera_link_optical_direct");
}

void sendEstCamLinkOpt2EstArLinkBaseTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( estCrntArCamPose.pose.position.x, 
                                    estCrntArCamPose.pose.position.y, 
                                    estCrntArCamPose.pose.position.z) );
  transform.setRotation( tf::Quaternion(estCrntArCamPose.pose.orientation.x, 
                                        estCrntArCamPose.pose.orientation.y,
                                        estCrntArCamPose.pose.orientation.z,
                                        estCrntArCamPose.pose.orientation.w) );
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "est_camera_link_optical_direct", "est_aruco_link_base_direct") );
  ROS_INFO("est_camera_link_optical_direct -> est_aruco_link_base_direct");
}

void sendOdom2EstArLinkBaseTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( estCrntArOdomPose.pose.position.x, 
                                    estCrntArOdomPose.pose.position.y, 
                                    estCrntArOdomPose.pose.position.z) );
  transform.setRotation( tf::Quaternion(estCrntArOdomPose.pose.orientation.x, 
                                        estCrntArOdomPose.pose.orientation.y,
                                        estCrntArOdomPose.pose.orientation.z,
                                        estCrntArOdomPose.pose.orientation.w) );
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "odom", "est_aruco_link_base_inverse") );
  ROS_INFO("odom -> est_aruco_link_base_inverse");
}

void sendEstArLinkBase2EstCamLinkOptTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform_inverse;
  transform.setOrigin( tf::Vector3( estCrntArCamPose.pose.position.x, 
                                    estCrntArCamPose.pose.position.y, 
                                    estCrntArCamPose.pose.position.z) );
  transform.setRotation( tf::Quaternion(estCrntArCamPose.pose.orientation.x, 
                                        estCrntArCamPose.pose.orientation.y,
                                        estCrntArCamPose.pose.orientation.z,
                                        estCrntArCamPose.pose.orientation.w) );
  transform_inverse = transform.inverse();
  br.sendTransform( tf::StampedTransform(transform_inverse, ros::Time::now(), "est_aruco_link_base_inverse", "est_camera_link_optical_inverse") );
  ROS_INFO("est_aruco_link_base_inverse -> est_camera_link_optical_inverse");
}

void sendEstCamLinkOpt2EstCamLinkBaseTransform(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform_inverse;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(-0.7071067811865476, 0, 0, 0.7071067811865476) );
  transform_inverse = transform.inverse();
  br.sendTransform( tf::StampedTransform(transform_inverse, ros::Time::now(), "est_camera_link_optical_inverse", "est_camera_link_base_inverse") );
  ROS_INFO("est_camera_link_optical_inverse -> est_camera_link_base_inverse");
}

// едет маркер
// получаем текущее оценочное положение _маркера_ относительно ГСК
// odom -> est_camera_link_base -> est_camera_link_optical -> est_aruco_link_base -> odom
void getEstCrntArOdomPose(const tf::TransformListener& listener){
  if (isStop(velCrntAr)) return;
  sendOdom2EstCamLinkBaseTransform();
  sendEstCamLinkBase2EstCamLinkOptTransform();
  sendEstCamLinkOpt2EstArLinkBaseTransform();
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("odom", "est_aruco_link_base_direct", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  estCrntArOdomPose.pose.position.x    = transform.getOrigin().x();
  estCrntArOdomPose.pose.position.y    = transform.getOrigin().y();
  estCrntArOdomPose.pose.position.z    = transform.getOrigin().z();
  estCrntArOdomPose.pose.orientation.x = transform.getRotation().x();
  estCrntArOdomPose.pose.orientation.y = transform.getRotation().y();
  estCrntArOdomPose.pose.orientation.z = transform.getRotation().z();
  estCrntArOdomPose.pose.orientation.w = transform.getRotation().w();
}

// едет камера
// получаем текущее оценочное положение _камеры_ относительно ГСК
// odom -> est_aruco_link_base -> est_camera_link_optical -> est_camera_link_base -> odom
void getEstCrntCamOdomPose(const tf::TransformListener& listener){
  if (isStop(velCrntCam)) return;
  sendOdom2EstArLinkBaseTransform();
  sendEstArLinkBase2EstCamLinkOptTransform();
  sendEstCamLinkOpt2EstCamLinkBaseTransform();
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("odom", "est_camera_link_base_inverse", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  estCrntCamOdomPose.pose.position.x    = transform.getOrigin().x();
  estCrntCamOdomPose.pose.position.y    = transform.getOrigin().y();
  estCrntCamOdomPose.pose.position.z    = transform.getOrigin().z();
  estCrntCamOdomPose.pose.orientation.x = transform.getRotation().x();
  estCrntCamOdomPose.pose.orientation.y = transform.getRotation().y();
  estCrntCamOdomPose.pose.orientation.z = transform.getRotation().z();
  estCrntCamOdomPose.pose.orientation.w = transform.getRotation().w();
}

// отображение текущих положений _маркера_ и _камеры_ 
void showVdrkPose(){

  if      (!isStop(velCrntAr))  printf("\n-------------------------ЕДЕТ МАРКЕР-------------------\n");
  else if (!isStop(velCrntCam)) printf("\n-------------------------ЕДЕТ КАМЕРА-------------------\n");
  else                          printf("\n--------------------------ВСЕ СТОИТ--------------------\n");

  printf("######################################################\n");
  printf("[текущее оценочное положение _маркера_ относительно _камеры_]\n");
  printf("estCrntArCamPose.pose.position.x    = %f\n", estCrntArCamPose.pose.position.x);
  printf("estCrntArCamPose.pose.position.y    = %f\n", estCrntArCamPose.pose.position.y);
  printf("estCrntArCamPose.pose.position.z    = %f\n", estCrntArCamPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _маркера_ относительно _камеры_]\n");
  printf("crntArCamPose.pose.position.x       = %f\n", crntArCamPose.pose.position.x);
  printf("crntArCamPose.pose.position.y       = %f\n", crntArCamPose.pose.position.y);
  printf("crntArCamPose.pose.position.z       = %f\n", crntArCamPose.pose.position.z);
  printf("\n######################################################\n");
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
  printf("[текущее оценочное положение _камеры_ относительно ГСК]\n");
  printf("estCrntCamOdomPose.pose.position.x  = %f\n", estCrntCamOdomPose.pose.position.x);
  printf("estCrntCamOdomPose.pose.position.y  = %f\n", estCrntCamOdomPose.pose.position.y);
  printf("estCrntCamOdomPose.pose.position.z  = %f\n", estCrntCamOdomPose.pose.position.z);
  printf("------------------------------------------------------\n");
  printf("[текущее фактическое положение _камеры_ относительно ГСК]\n");
  printf("crntCamOdomPose.pose.position.x     = %f\n", crntCamOdomPose.pose.position.x);
  printf("crntCamOdomPose.pose.position.y     = %f\n", crntCamOdomPose.pose.position.y);
  printf("crntCamOdomPose.pose.position.z     = %f\n", crntCamOdomPose.pose.position.z);
  printf("######################################################\n\n");
}

void setRes2Write(){
  vdrkPose.estCrntArCamPose    = estCrntArCamPose;
  vdrkPose.crntArCamPose       = crntArCamPose;
  vdrkPose.estCrntArOdomPose   = estCrntArOdomPose;
  vdrkPose.crntArOdomPose      = crntArOdomPose;
  vdrkPose.estCrntCamOdomPose  = estCrntCamOdomPose;
  vdrkPose.crntCamOdomPose     = crntCamOdomPose;
}

// расстояние между точками
double getDistance(const double t1, const double t2) {                
  return t1 - t2;
}

void findArNewPose2OldPoseOffset(geometry_msgs::Vector3& arNewPose2OldPoseOffset){
  arNewPose2OldPoseOffset.x = abs(getDistance(estCrntArCamPose.pose.position.x, estPrevArCamPose.pose.position.x));
  arNewPose2OldPoseOffset.y = abs(getDistance(estCrntArCamPose.pose.position.y, estPrevArCamPose.pose.position.y));
  arNewPose2OldPoseOffset.z = abs(getDistance(estCrntArCamPose.pose.position.z, estPrevArCamPose.pose.position.z));
}

// записываем результаты эксперимента
void sendVdrkPoseToTopic(){
  geometry_msgs::Vector3 arNewPose2OldPoseOffset;
  findArNewPose2OldPoseOffset(arNewPose2OldPoseOffset);
  double lenOfArNewPose2OldPoseOffset = std::sqrt(std::pow(arNewPose2OldPoseOffset.x, 2) +
                                                  std::pow(arNewPose2OldPoseOffset.y, 2) +
                                                  std::pow(arNewPose2OldPoseOffset.z, 2));
  if (lenOfArNewPose2OldPoseOffset > MEASURE_STEP){
    estPrevArCamPose = estCrntArCamPose;
    setRes2Write();
    vdrkPosePub.publish(vdrkPose);
  }
}

void setup(ros::NodeHandle& node){
  estCrntPoseSub        = node.subscribe("/aruco_single/pose",    0, getEstArCamPoseHandler);
  crntOdomPoseSub       = node.subscribe("/gazebo/model_states",  0, getCrntOdomPoseHandler);
  velCrntArSub          = node.subscribe("/aruco_cmd_vel",        0, getVelCrntArHandler);
  velCrntCamSub         = node.subscribe("/camera_cmd_vel",       0, getVelCrntCamHandler);
  vdrkPosePub           = node.advertise<send_results_vdrk::VdrkPose> ("vdrkPoseTopic", 0);
}

// задаем Н.У. для ВДРК
void setInitVdrkPose(){
  if (!initVdrkPoseIsSet){
    initVdrkPoseIsSet     = true;
    estCrntArOdomPose     = crntArOdomPose;
    estCrntCamOdomPose    = crntCamOdomPose;
    estPrevArCamPose      = estCrntArCamPose;
    sendOdom2EstCamLinkBaseTransform();
    sendEstCamLinkBase2EstCamLinkOptTransform();
    sendEstCamLinkOpt2EstArLinkBaseTransform();
  }
}

// user test function
void setCam2ArInversePose(){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( estCrntArCamPose.pose.position.x, 
                                    estCrntArCamPose.pose.position.y, 
                                    estCrntArCamPose.pose.position.z) );
  transform.setRotation( tf::Quaternion(estCrntArCamPose.pose.orientation.x, 
                                        estCrntArCamPose.pose.orientation.y,
                                        estCrntArCamPose.pose.orientation.z,
                                        estCrntArCamPose.pose.orientation.w) );
  printf("\n------------------------------------------");
  printf("\ngetOrigin().x()   = %.5f", transform.getOrigin().x());
  printf("\ngetOrigin().y()   = %.5f", transform.getOrigin().y());
  printf("\ngetOrigin().z()   = %.5f", transform.getOrigin().z());
  printf("\ngetRotation().x() = %.5f", transform.getRotation().x());
  printf("\ngetRotation().y() = %.5f", transform.getRotation().y());
  printf("\ngetRotation().z() = %.5f", transform.getRotation().z());
  printf("\ngetRotation().w() = %.5f", transform.getRotation().w());
  printf("\n------------------------------------------");
  printf("\ninverse().getOrigin().x()   = %.5f", transform.inverse().getOrigin().x());
  printf("\ninverse().getOrigin().y()   = %.5f", transform.inverse().getOrigin().y());
  printf("\ninverse().getOrigin().z()   = %.5f", transform.inverse().getOrigin().z());
  printf("\ninverse().getRotation().x() = %.5f", transform.inverse().getRotation().x());
  printf("\ninverse().getRotation().y() = %.5f", transform.inverse().getRotation().y());
  printf("\ninverse().getRotation().z() = %.5f", transform.inverse().getRotation().z());
  printf("\ninverse().getRotation().w() = %.5f", transform.inverse().getRotation().w());
  printf("\n------------------------------------------");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "vdrk_pose_pub");
  ROS_INFO_STREAM("vdrk_pose_pub is ready");
  ros::NodeHandle node;
  tf::TransformListener listener;
  setup(node);
  ros::Rate loop_rate(30);
  while (ros::ok()){
    ros::spinOnce();
    allPosesGet = getEstCrntArCamPose && getOdomPoses;
    if (!allPosesGet) continue;
    getEstCrntArCamPose = false;
    getOdomPoses        = false;
    // setCam2ArInversePose();
    setInitVdrkPose();
    getCrntArCamPose(listener);
    getEstCrntArOdomPose(listener);
    getEstCrntCamOdomPose(listener);
    showVdrkPose();
    sendVdrkPoseToTopic();
    loop_rate.sleep();
  }
  return 0;
}