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

#define MIN_ROBOTS_DIST 2                                         // минимальное  расстояние между turtle_front & turtle_back
#define MAX_ROBOTS_DIST 8                                         // максимальное расстояние между turtle_front & turtle_back

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее  положение маркера относительно камеры
geometry_msgs::PoseStamped desiredArucoCameraPose;                // желаемое положение маркера относительно камеры

// geometry_msgs::PoseStamped currentTurtleFrontOdomPose;            // текущее  положение turtle_front относительно ГСК
// geometry_msgs::PoseStamped desiredTurtleFrontOdomPose;            // желаемое положение turtle_front относительно ГСК

geometry_msgs::Twist velCmd2Turtle;                               // скорость для роботов [м/с]
ros::Publisher velCmd2TurtleBack;                                 // скорость для заднего робота
ros::Publisher velCmd2TurtleFront;                                // скорость для переднего робота

double distance_between_robots;                                   // расстояние между маркером и камерой

enum TurtleType{
  TURTLE_FRONT,
  TURTLE_BACK
};

// double goal_x = 0;
// double goal_y = 0;
// double goal_z = 0;
double velocity = 0;
double move_precision = 0;

bool turtle_back_stop = true;
bool turtle_front_stop = true;

bool allCallbacksCall = false;                                    // все колбеки вызваны
bool getEstimateCurrentArucoCameraPoseCallback = false;
bool getCurrentTurtleFrontOdomPoseCallback = false;

// bool goal_x_reached = false;
// bool goal_y_reached = false;
// bool goal_z_reached = false;

void setup() {
 
  // desiredTurtleFrontOdomPose.pose.position.x = goal_x;
  // desiredTurtleFrontOdomPose.pose.position.y = goal_y;
  // desiredTurtleFrontOdomPose.pose.position.z = goal_z;

  velCmd2Turtle.linear.x = 0.0;
  velCmd2Turtle.linear.y = 0.0;
  velCmd2Turtle.linear.z = 0.0;
  velCmd2Turtle.angular.x = 0.0;
  velCmd2Turtle.angular.y = 0.0;
  velCmd2Turtle.angular.z = 0.0;
}

void stop_turtle(size_t type){
  velCmd2Turtle.linear.x = 0.0;
  velCmd2Turtle.linear.y = 0.0;
  velCmd2Turtle.linear.z = 0.0;
  velCmd2Turtle.angular.x = 0.0;
  velCmd2Turtle.angular.y = 0.0;
  velCmd2Turtle.angular.z = 0.0;

  if (type == TURTLE_FRONT){
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = true;
  }

  if (type == TURTLE_BACK){
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = true;
  }
  
}

double getDistanceToGoal(const double desired, const double current) {
  return desired - current;
}

void setTurtleVelocity(size_t type) {

  double distanceToGoal_x;
  double distanceToGoal_y;
  double distanceToGoal_z;

  if (type == TURTLE_FRONT) {

    desiredArucoCameraPose = estimateCurrentArucoCameraPose;
    desiredArucoCameraPose.pose.position.z = estimateCurrentArucoCameraPose.pose.position.z + MAX_ROBOTS_DIST;

    distanceToGoal_x = ( 1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.x, estimateCurrentArucoCameraPose.pose.position.x);
    distanceToGoal_y = ( 1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.z, estimateCurrentArucoCameraPose.pose.position.z);
    distanceToGoal_z = (-1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.y, estimateCurrentArucoCameraPose.pose.position.y);
  }

  if (type == TURTLE_BACK) {

    desiredArucoCameraPose = estimateCurrentArucoCameraPose;
    desiredArucoCameraPose.pose.position.z = estimateCurrentArucoCameraPose.pose.position.z - MIN_ROBOTS_DIST;

    distanceToGoal_x = (-1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.z, estimateCurrentArucoCameraPose.pose.position.z);
    distanceToGoal_y = (-1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.x, estimateCurrentArucoCameraPose.pose.position.x);
    distanceToGoal_z = ( 1) * getDistanceToGoal(desiredArucoCameraPose.pose.position.y, estimateCurrentArucoCameraPose.pose.position.y);
  }

  ROS_INFO("\n"
          "distanceToGoal_x = %.5f\n"
          "distanceToGoal_y = %.5f\n"
          "distanceToGoal_z = %.5f",
          distanceToGoal_x, distanceToGoal_y, distanceToGoal_z);

  //velocity x
  if (abs(distanceToGoal_x) > move_precision) {
    velCmd2Turtle.linear.x = velocity * distanceToGoal_x / abs(distanceToGoal_x);
  }
  else {
    if (type != TURTLE_BACK){
      ROS_INFO("goal_x_reached!");
      //goal_x_reached = true;
    }
    velCmd2Turtle.linear.x = 0;
  }

  //velocity y
  if (abs(distanceToGoal_y) > move_precision) {
    velCmd2Turtle.linear.y = velocity * distanceToGoal_y / abs(distanceToGoal_y);
  }
  else {
    if (type != TURTLE_BACK){
      ROS_INFO("goal_y_reached!");
      //goal_y_reached = true;
    }
    velCmd2Turtle.linear.y = 0;
  }

  //velocity z
  if (abs(distanceToGoal_z) > move_precision) {
    velCmd2Turtle.linear.z = velocity * distanceToGoal_z / abs(distanceToGoal_z);
  }
  else {
    if (type != TURTLE_BACK){
      ROS_INFO("goal_z_reached!");
      //goal_z_reached = true;
    }
    velCmd2Turtle.linear.z = 0;
  }
}


void go_turtle_front(){

  if (distance_between_robots < MAX_ROBOTS_DIST && turtle_back_stop) {
    setTurtleVelocity(TURTLE_FRONT);
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = false;
  } else {
    stop_turtle(TURTLE_FRONT);
  }

}

void go_turtle_back(){

  if (distance_between_robots > MIN_ROBOTS_DIST && turtle_front_stop) {
    setTurtleVelocity(TURTLE_BACK);
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = false;
  } else {
    stop_turtle(TURTLE_BACK);
  }

}

// // тек позиция переднего робота
// void getCurrentTurtleFrontOdomPose(const gazebo_msgs::ModelStates& turtleFrontMsg) {
//   currentTurtleFrontOdomPose.pose.position.x = turtleFrontMsg.pose[1].position.x;
//   currentTurtleFrontOdomPose.pose.position.y = turtleFrontMsg.pose[1].position.y;
//   currentTurtleFrontOdomPose.pose.position.z = turtleFrontMsg.pose[1].position.z;

//   currentTurtleFrontOdomPose.pose.orientation.w = turtleFrontMsg.pose[1].orientation.w;
//   currentTurtleFrontOdomPose.pose.orientation.x = turtleFrontMsg.pose[1].orientation.x;
//   currentTurtleFrontOdomPose.pose.orientation.y = turtleFrontMsg.pose[1].orientation.y;
//   currentTurtleFrontOdomPose.pose.orientation.z = turtleFrontMsg.pose[1].orientation.z;

//   getCurrentTurtleFrontOdomPoseCallback = true;
// }

// тек оценка маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

void go_to_goal(){

  // расстояние между маркером и камерой
  distance_between_robots = std::sqrt(std::pow(estimateCurrentArucoCameraPose.pose.position.x, 2) + 
                                      std::pow(estimateCurrentArucoCameraPose.pose.position.y, 2) + 
                                      std::pow(estimateCurrentArucoCameraPose.pose.position.z, 2));
  ROS_INFO("\ndistance_between_robots = %.5f\n", distance_between_robots);
  go_turtle_front();
  go_turtle_back();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "vdrk_move");

  ros::NodeHandle node;
  tf::TransformListener listener;

  // ros::param::param<double>("~Goal_x", goal_x, 0);
  // ros::param::param<double>("~Goal_y", goal_y, 0);
  // ros::param::param<double>("~Goal_z", goal_z, 0);
  ros::param::param<double>("~Move_precision", move_precision, 0);
  ros::param::param<double>("~Velocity", velocity, 0);

  setup();

  // ros::Subscriber currentTurtleFrontOdomPoseSub =                                               // тек позиция переднего робота
  //   node.subscribe("/gazebo/model_states", 0, getCurrentTurtleFrontOdomPose);

  ros::Subscriber arucoCameraPoseSub =
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);                 // тек оценка маркера относительно камеры

  velCmd2TurtleBack =
    node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);                                 // скорость для заднего робота

  velCmd2TurtleFront =
    node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel", 0);                                  // скорость для переднего робота

  ros::Rate loop_rate(30);

  while (ros::ok()/* && (!goal_x_reached || !goal_y_reached || !goal_z_reached)*/) {

    ros::spinOnce();

    allCallbacksCall = getEstimateCurrentArucoCameraPoseCallback &&
                          getCurrentTurtleFrontOdomPoseCallback;

    if (!getEstimateCurrentArucoCameraPoseCallback) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;
    getCurrentTurtleFrontOdomPoseCallback     = false;

    go_to_goal();

    loop_rate.sleep();
  }

  ROS_INFO("\n"
          "goal_x has been reached!\n"
          "goal_y has been reached!\n"
          "goal_z has been reached!\n");

  return 0;
}