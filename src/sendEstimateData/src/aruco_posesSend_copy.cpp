#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <turtlesim/Pose.h>
#include "sendEstimateData/Poses.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

#define MEASURE_STEP 0.01 // [м] = 1 см - шаг измерения положения маркера

sendEstimateData::Poses arucoPoses2Write;
ros::Publisher arucoResDataBagPub;
ros::Publisher cmdVelPub;
ros::Publisher movePub;

//скорость для черепашки
geometry_msgs::Twist cmdVel;

//положения маркера и черепашки
geometry_msgs::PoseStamped estimateCurrentArucoCameraPose, estimateCurrentArucoOdomPose, //текущая оценка положения маркера относительно камеры и мира, 
                           estimatePrevArucoOdomPose, tmp_ArucoOdomPose, measure_ArucoGazeboOdomPose; //предыдущая оценка положения маркера относительно мира, временная переменная,  положение маркера для вывода в файл 

gazebo_msgs::ModelStates currentArucoGazeboOdomPose, prevArucoGazeboOdomPose, //текущее и предыдущее фактическое положение маркера относительно мира,
                          tmp_ArucoGazeboOdomPose; //временная переменная,
                                                                              
turtlesim::Pose currentTurtlePose, prevTurtlePose, tmp_TurtlePose; //текущее и предыдущее положение черепашки, временная переменная

bool arucoStartPose = true; //маркер еще не двигался
bool turtleStartPose = true; //черепешка еще не двигалась

bool allCallbacksCall = false; //все колбеки вызваны
bool getEstimateCurrentArucoCameraPoseCallback = false;
bool getCurrentArucoGazeboOdomPoseCallback = false;
bool getCurrentTurtlePoseCallback = false;

geometry_msgs::Pose2D passedTurtleWay; //пройденный путь черепашкой
geometry_msgs::Vector3 estimatePassedArucoWay, truePassedArucoWay, estimateMarkerMoving; //оценочный и фактический пройденный путь маркером
                                                                                         //оценочное перемещение маркера
double velocity_coeff = 0.0; //коэф. скорости черепашки

void setup() {

  //aruco pose
  estimateCurrentArucoCameraPose.pose.position.x = 0.0;
  estimateCurrentArucoCameraPose.pose.position.y = 0.0;
  estimateCurrentArucoCameraPose.pose.position.z = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.w = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.x = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.y = 0.0;
  estimateCurrentArucoCameraPose.pose.orientation.z = 0.0;

  estimateCurrentArucoOdomPose = estimateCurrentArucoCameraPose;
  estimatePrevArucoOdomPose = estimateCurrentArucoCameraPose;
  tmp_ArucoOdomPose = estimateCurrentArucoCameraPose;

  //turtle pose
  currentTurtlePose.x = 0.0;
  currentTurtlePose.y = 0.0;
  currentTurtlePose.theta = 0.0;

  prevTurtlePose = currentTurtlePose;
  tmp_TurtlePose = currentTurtlePose;

  //path
  passedTurtleWay.x = 0.0;
  passedTurtleWay.y = 0.0;

  estimatePassedArucoWay.x = 0.0;
  estimatePassedArucoWay.y = 0.0;
  estimatePassedArucoWay.z = 0.0;
  truePassedArucoWay = estimatePassedArucoWay;
  estimateMarkerMoving = estimatePassedArucoWay;

  //start velocity
  cmdVel.linear.x = 0.0;
  cmdVel.linear.y = 0.0;
  cmdVel.linear.z = 0.0;
  cmdVel.angular.x = 0.0;
  cmdVel.angular.y = 0.0;
  cmdVel.angular.z = 0.0;
}

void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { //получаем текущую оценку положения маркера относительно камеры
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

void getCurrentArucoGazeboOdomPose(const gazebo_msgs::ModelStates& arucoGazeboMsg) { //получаем текущее фактическое положение моделей относительно мира
  currentArucoGazeboOdomPose = arucoGazeboMsg;
  getCurrentArucoGazeboOdomPoseCallback = true;
}

void getCurrentTurtlePose(const turtlesim::Pose& turtleMsg) { //получаем текущее положение черепашки
  currentTurtlePose = turtleMsg;
  getCurrentTurtlePoseCallback = true;
}

void transformPose(const tf::TransformListener& listener){ //перевод оценки положения маркера относительно камеры 
                                                           //в оценку положения маркера относительно мира 
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("odom", "aruco_marker_frame",
                              ros::Time(0), transform);
    
    estimateCurrentArucoOdomPose.header.frame_id = "odom";
    estimateCurrentArucoOdomPose.pose.position.x = transform.getOrigin().x();
    estimateCurrentArucoOdomPose.pose.position.y = transform.getOrigin().y();
    estimateCurrentArucoOdomPose.pose.position.z = transform.getOrigin().z();
    estimateCurrentArucoOdomPose.pose.orientation.w = transform.getRotation().getW();
    estimateCurrentArucoOdomPose.pose.orientation.x = transform.getRotation().getX();
    estimateCurrentArucoOdomPose.pose.orientation.y = transform.getRotation().getY();
    estimateCurrentArucoOdomPose.pose.orientation.z = transform.getRotation().getZ();
    
    ROS_INFO("\n"
              "camera_frame:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)----->\n"
              "        odom:\tposition: (%.5f, %.5f, %.5f)\torientation: (%.5f, %.5f, %.5f, %.5f)",
      estimateCurrentArucoCameraPose.pose.position.x, 
      estimateCurrentArucoCameraPose.pose.position.y, estimateCurrentArucoCameraPose.pose.position.z,
      estimateCurrentArucoCameraPose.pose.orientation.w, estimateCurrentArucoCameraPose.pose.orientation.x,
      estimateCurrentArucoCameraPose.pose.orientation.y, estimateCurrentArucoCameraPose.pose.orientation.z,
      
      estimateCurrentArucoOdomPose.pose.position.x, 
      estimateCurrentArucoOdomPose.pose.position.y, estimateCurrentArucoOdomPose.pose.position.z,
      estimateCurrentArucoOdomPose.pose.orientation.w, estimateCurrentArucoOdomPose.pose.orientation.x,
      estimateCurrentArucoOdomPose.pose.orientation.y, estimateCurrentArucoOdomPose.pose.orientation.z);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void updateArucoPose(){ //обновляем оценочное предыдущее положение маркера

  if (arucoStartPose){
    estimatePrevArucoOdomPose = estimateCurrentArucoOdomPose;
    tmp_ArucoOdomPose = estimateCurrentArucoOdomPose;
  } else {
    estimatePrevArucoOdomPose = tmp_ArucoOdomPose;
    tmp_ArucoOdomPose = estimateCurrentArucoOdomPose;
  }
}

void updateTurtlePose(){ //обновляем предыдущее положение черепашки
  
  if (turtleStartPose){
    prevTurtlePose = currentTurtlePose;
    tmp_TurtlePose = currentTurtlePose;
    turtleStartPose = false;
  } else {
    prevTurtlePose = tmp_TurtlePose;
    tmp_TurtlePose = currentTurtlePose;
  }
}

double getDistance(const double currentCoordinate, const double prevCoordinate) { //расстояние между точками
  return currentCoordinate - prevCoordinate;
}

void updatePassedPath(){ //обновляем суммарные пройденные пути

  //turtle path
  passedTurtleWay.x += abs(getDistance(currentTurtlePose.x, prevTurtlePose.x));
  passedTurtleWay.y += abs(getDistance(currentTurtlePose.y, prevTurtlePose.y));

  //aruco estimate path
  estimatePassedArucoWay.x += abs(getDistance(estimateCurrentArucoOdomPose.pose.position.x, estimatePrevArucoOdomPose.pose.position.x));
  estimatePassedArucoWay.y += abs(getDistance(estimateCurrentArucoOdomPose.pose.position.y, estimatePrevArucoOdomPose.pose.position.y));
  estimatePassedArucoWay.z += abs(getDistance(estimateCurrentArucoOdomPose.pose.position.z, estimatePrevArucoOdomPose.pose.position.z));
}

void setVelocity() { //устанавливаем скорость черепашке
  cmdVel.linear.x = velocity_coeff * getDistance(estimateCurrentArucoOdomPose.pose.position.x, estimatePrevArucoOdomPose.pose.position.x);
  cmdVel.linear.y = velocity_coeff * getDistance(estimateCurrentArucoOdomPose.pose.position.y, estimatePrevArucoOdomPose.pose.position.y);
  cmdVelPub.publish(cmdVel);
}

void fillArucoPoses2Write(){
  //оценочное положение
  arucoPoses2Write.estimatePose = estimateCurrentArucoOdomPose;

  //фактическое положение
  arucoPoses2Write.truePose.pose.position.x = currentArucoGazeboOdomPose.pose[1].position.x;
  arucoPoses2Write.truePose.pose.position.y = currentArucoGazeboOdomPose.pose[1].position.y;
  arucoPoses2Write.truePose.pose.position.z = currentArucoGazeboOdomPose.pose[1].position.z;

  arucoPoses2Write.truePose.pose.orientation.w = currentArucoGazeboOdomPose.pose[1].orientation.w;
  arucoPoses2Write.truePose.pose.orientation.x = currentArucoGazeboOdomPose.pose[1].orientation.x;
  arucoPoses2Write.truePose.pose.orientation.y = currentArucoGazeboOdomPose.pose[1].orientation.y;
  arucoPoses2Write.truePose.pose.orientation.z = currentArucoGazeboOdomPose.pose[1].orientation.z;
}

void writeArucoPoseData2Bag(){ //записываем фактическое и оценочное положение маркера
  
  if (arucoStartPose){
    measure_ArucoGazeboOdomPose = estimateCurrentArucoOdomPose;
    arucoStartPose = false;
  }
  
  //оценочное перемещение маркера
  estimateMarkerMoving.x = abs(getDistance(estimateCurrentArucoOdomPose.pose.position.x, measure_ArucoGazeboOdomPose.pose.position.x));
  estimateMarkerMoving.y = abs(getDistance(estimateCurrentArucoOdomPose.pose.position.y, measure_ArucoGazeboOdomPose.pose.position.y));
  estimateMarkerMoving.z = abs(getDistance(estimateCurrentArucoOdomPose.pose.position.z, measure_ArucoGazeboOdomPose.pose.position.z));
  double trueMarkerSpaceMoving = std::sqrt(std::pow(estimateMarkerMoving.x, 2) + std::pow(estimateMarkerMoving.y, 2) + std::pow(estimateMarkerMoving.z, 2));

  if (trueMarkerSpaceMoving > MEASURE_STEP) {
    
    std_msgs::Float64 moveMsg;
    moveMsg.data = trueMarkerSpaceMoving;
    movePub.publish(moveMsg);

    //обновляем точку измерения положения маркера
    measure_ArucoGazeboOdomPose = estimateCurrentArucoOdomPose; 

    fillArucoPoses2Write();
    arucoResDataBagPub.publish(arucoPoses2Write);
  }
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "aruco_listener");

  ros::NodeHandle node;
  tf::TransformListener listener;
  // ros::param::param<double>("~Velocity_coeff", velocity_coeff, 0.0);

  setup();

  ros::Subscriber arucoCameraPoseSub =
    node.subscribe("/aruco_single/pose", 100, getEstimateCurrentArucoCameraPose);

  ros::Subscriber arucoGazeboPoseSub =
    node.subscribe("/gazebo/model_states", 100, getCurrentArucoGazeboOdomPose);

  ros::Subscriber turtlePoseSub =
    node.subscribe("/turtle1/pose", 100, getCurrentTurtlePose);

  arucoResDataBagPub =
    node.advertise<sendEstimateData::Poses>("arucoResDataBagTopic", 100);

  cmdVelPub =
    node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    
  movePub =
    node.advertise<std_msgs::Float64>("move", 100);

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    allCallbacksCall = getEstimateCurrentArucoCameraPoseCallback &&
                        getCurrentArucoGazeboOdomPoseCallback    &&
                        getCurrentTurtlePoseCallback;

    if (allCallbacksCall){
      transformPose(listener);
      updateArucoPose();
      updateTurtlePose();
      updatePassedPath();
      setVelocity();
      writeArucoPoseData2Bag();
    
      ROS_INFO("\nestimateCurrentArucoOdomPose.pose.position.x = %.5f \n"
                "estimateCurrentArucoOdomPose.pose.position.y = %.5f \n"
                "estimateCurrentArucoOdomPose.pose.position.z = %.5f \n"

                "currentArucoGazeboOdomPose.pose[1].position.x = %.5f \n"
                "currentArucoGazeboOdomPose.pose[1].position.y = %.5f \n"
                "currentArucoGazeboOdomPose.pose[1].position.z = %.5f \n"

                "passedTurtleWay.x = %.5f m \n"
                "passedTurtleWay.y = %.5f m \n"

                "estimatePassedArucoWay.x = %.5f m \n"
                "estimatePassedArucoWay.y = %.5f m \n"
                "estimatePassedArucoWay.z = %.5f m \n\n",

                estimateCurrentArucoOdomPose.pose.position.x,
                estimateCurrentArucoOdomPose.pose.position.y,
                estimateCurrentArucoOdomPose.pose.position.z,

                currentArucoGazeboOdomPose.pose[1].position.x,
                currentArucoGazeboOdomPose.pose[1].position.y,
                currentArucoGazeboOdomPose.pose[1].position.z,
                
                passedTurtleWay.x,
                passedTurtleWay.y,
                
                estimatePassedArucoWay.x,
                estimatePassedArucoWay.y,
                estimatePassedArucoWay.z);
    }
              
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}