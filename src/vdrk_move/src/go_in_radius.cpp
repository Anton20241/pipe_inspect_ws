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

#define MIN_ROBOTS_DIST 0.8                                       // минимальное  расстояние между turtle_front & turtle_back
#define MAX_ROBOTS_DIST 1.1                                       // максимальное расстояние между turtle_front & turtle_back

geometry_msgs::PoseStamped estimateCurrentArucoCameraPose;        // текущее оц. положение маркера относительно камеры
geometry_msgs::PoseStamped startCurrentTurtleFrontOdomPose;       // начальное   положение маркера относительно ГСК
geometry_msgs::PoseStamped currentTurtleFrontOdomPose;            // текущее     положение маркера относительно ГСК
geometry_msgs::PoseStamped currentTurtleBackOdomPose;             // текущее     положение камеры  относительно ГСК

geometry_msgs::Twist velCmd2Turtle;                               // скорость для роботов [м/с]
ros::Publisher velCmd2TurtleBack;                                 // скорость для заднего робота
ros::Publisher velCmd2TurtleFront;                                // скорость для переднего робота

double distance_between_robots;                                   // расстояние между маркером и камерой

enum TurtleType{
  TURTLE_FRONT,
  TURTLE_BACK
};

double velocity             = 0;
double move_precision       = 0;
double radius_tube          = 0;                                  // [м] - радиус кривизны оси трубы

bool turtle_back_stop       = true;
bool turtle_front_stop      = true;
bool turtle_front_start     = true;
bool turtle_back_start      = true;

bool goal_x_reached         = false;
bool goal_y_reached         = false;

bool allCallbacksCall                          = false;           // все колбеки вызваны
bool getEstimateCurrentArucoCameraPoseCallback = false;
bool getCurrentTurtleFrontOdomPoseCallback     = false;
bool getCurrentTurtleBackOdomPoseCallback      = false;

void setup() {
  velCmd2Turtle.linear.x  = 0.0;
  velCmd2Turtle.linear.y  = 0.0;
  velCmd2Turtle.linear.z  = 0.0;
  velCmd2Turtle.angular.x = 0.0;
  velCmd2Turtle.angular.y = 0.0;
  velCmd2Turtle.angular.z = 0.0;
}

void stop_turtle(size_t type){
  velCmd2Turtle.linear.x  = 0.0;
  velCmd2Turtle.linear.y  = 0.0;
  velCmd2Turtle.linear.z  = 0.0;
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

double getDistance(const double desired, const double current) {
  return desired - current;
}

void go_turtle_front(){

  // if (turtle_front_start){
  //   startCurrentTurtleFrontOdomPose = currentTurtleFrontOdomPose;
  //   turtle_front_start = false;
  // }

  if ((distance_between_robots <= MAX_ROBOTS_DIST) && turtle_back_stop) {
    velCmd2Turtle.linear.y  = velocity;
    velCmd2Turtle.angular.z = (-1.0) * velocity / radius_tube;   // [rad/s] - omega
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = false;

  } else {
    velCmd2Turtle.linear.y  = 0;
    velCmd2Turtle.angular.z = 0;
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = true;
    
  }
}

void go_turtle_back(){
  
  // if(turtle_back_start && turtle_front_stop){
  //   if (!goal_x_reached || !goal_y_reached){
  //     go_turtle_back_to_start();
  //     turtle_back_stop  = false;
  //   } else {
  //     turtle_back_start = false;
  //   }

  // } else 
  
  if ((distance_between_robots >= MIN_ROBOTS_DIST) && turtle_front_stop) {
    velCmd2Turtle.linear.y  = velocity;
    velCmd2Turtle.angular.z = (-1.0) * velocity / radius_tube;   // [rad/s] - omega
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = false;

  } else {
    velCmd2Turtle.linear.y  = 0;
    velCmd2Turtle.angular.z = 0;
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = true;
  }
}

void go_vdrk_in_radius(){

  // расстояние между маркером и камерой
  distance_between_robots = std::sqrt(std::pow(estimateCurrentArucoCameraPose.pose.position.x, 2) + 
                                      std::pow(estimateCurrentArucoCameraPose.pose.position.y, 2) + 
                                      std::pow(estimateCurrentArucoCameraPose.pose.position.z, 2));
  ROS_INFO("\ndistance_between_robots = %.5f\n", distance_between_robots);
  go_turtle_front();
  go_turtle_back();
}

// тек оценка маркера относительно камеры
void getEstimateCurrentArucoCameraPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estimateCurrentArucoCameraPose            = arucoCameraMsg;
  getEstimateCurrentArucoCameraPoseCallback = true;
}

// тек позиция маркера относительно ГСК
void getCurrentTurtleFrontOdomPose(const gazebo_msgs::ModelStates& turtleFrontMsg) {
  currentTurtleFrontOdomPose.pose.position.x    = turtleFrontMsg.pose[1].position.x;
  currentTurtleFrontOdomPose.pose.position.y    = turtleFrontMsg.pose[1].position.y;
  currentTurtleFrontOdomPose.pose.position.z    = turtleFrontMsg.pose[1].position.z;
  currentTurtleFrontOdomPose.pose.orientation.w = turtleFrontMsg.pose[1].orientation.w;
  currentTurtleFrontOdomPose.pose.orientation.x = turtleFrontMsg.pose[1].orientation.x;
  currentTurtleFrontOdomPose.pose.orientation.y = turtleFrontMsg.pose[1].orientation.y;
  currentTurtleFrontOdomPose.pose.orientation.z = turtleFrontMsg.pose[1].orientation.z;

  getCurrentTurtleFrontOdomPoseCallback = true;
}

// тек позиция камеры относительно ГСК
void getCurrentTurtleBackOdomPose(const gazebo_msgs::ModelStates& turtleBackMsg) {
  currentTurtleBackOdomPose.pose.position.x    = turtleBackMsg.pose[2].position.x;
  currentTurtleBackOdomPose.pose.position.y    = turtleBackMsg.pose[2].position.y;
  currentTurtleBackOdomPose.pose.position.z    = turtleBackMsg.pose[2].position.z;
  currentTurtleBackOdomPose.pose.orientation.w = turtleBackMsg.pose[2].orientation.w;
  currentTurtleBackOdomPose.pose.orientation.x = turtleBackMsg.pose[2].orientation.x;
  currentTurtleBackOdomPose.pose.orientation.y = turtleBackMsg.pose[2].orientation.y;
  currentTurtleBackOdomPose.pose.orientation.z = turtleBackMsg.pose[2].orientation.z;

  getCurrentTurtleBackOdomPoseCallback = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "vdrk_move_radius");
  ros::NodeHandle node;
  ros::param::param<double>("~Move_precision", move_precision, 0);
  ros::param::param<double>("~Velocity",       velocity,       0);
  ros::param::param<double>("~Radius_tube",    radius_tube,    0);

  setup();

  ros::Subscriber currentTurtleFrontOdomPoseSub =                                   // тек позиция маркера относительно ГСК
    node.subscribe("/gazebo/model_states", 0, getCurrentTurtleFrontOdomPose);

  ros::Subscriber currentTurtleBackOdomPoseSub =                                    // тек позиция камеры относительно ГСК
    node.subscribe("/gazebo/model_states", 0, getCurrentTurtleBackOdomPose);

  ros::Subscriber arucoCameraPoseSub =
    node.subscribe("/aruco_single/pose", 0, getEstimateCurrentArucoCameraPose);     // тек оценка маркера относительно камеры

  velCmd2TurtleBack =
    node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);                     // скорость для заднего робота

  velCmd2TurtleFront =
    node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel", 0);                      // скорость для переднего робота

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    ros::spinOnce();

    allCallbacksCall =  getEstimateCurrentArucoCameraPoseCallback &&
                        getCurrentTurtleBackOdomPoseCallback      &&
                        getCurrentTurtleFrontOdomPoseCallback;

    if (!allCallbacksCall) continue;
    getEstimateCurrentArucoCameraPoseCallback = false;
    getCurrentTurtleBackOdomPoseCallback      = false;
    getCurrentTurtleFrontOdomPoseCallback     = false;
    
    go_vdrk_in_radius();

    loop_rate.sleep();
  }

  return 0;
}