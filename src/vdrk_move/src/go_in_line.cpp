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

geometry_msgs::PoseStamped estCrntCamZrPntPose;                   // текущее оц. положение маркера относительно камеры
geometry_msgs::PoseStamped desiredCamZrPntPose;                   // желаемое положение маркера относительно камеры

// geometry_msgs::PoseStamped currentTurtleFrontOdomPose;         // текущее  положение turtle_front относительно ГСК
// geometry_msgs::PoseStamped desiredTurtleFrontOdomPose;         // желаемое положение turtle_front относительно ГСК

geometry_msgs::Twist velCmd2Turtle;                               // скорость для роботов [м/с]
ros::Publisher velCmd2TurtleBack;                                 // скорость для заднего робота
ros::Publisher velCmd2TurtleFront;                                // скорость для переднего робота

double distance_between_robots;                                   // расстояние между маркером и камерой

enum TurtleType{
  TURTLE_FRONT,
  TURTLE_BACK
};

double velocity = 0;

bool turtle_back_stop     = true;
bool turtle_front_stop    = true;
bool getPoseEstZrPnt      = false;

void setup() {
  velCmd2Turtle.linear.x  = 0.0;
  velCmd2Turtle.linear.y  = 0.0;
  velCmd2Turtle.linear.z  = 0.0;
  velCmd2Turtle.angular.x = 0.0;
  velCmd2Turtle.angular.y = 0.0;
  velCmd2Turtle.angular.z = 0.0;
}

void go_turtle_front(){

  if ((distance_between_robots < MAX_ROBOTS_DIST) && turtle_back_stop) {
    velCmd2Turtle.linear.y  = velocity;
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = false;
  } else {
    velCmd2Turtle.linear.y  = 0;
    velCmd2TurtleFront.publish(velCmd2Turtle);
    turtle_front_stop = true;
  }
}

void go_turtle_back(){

  if ((distance_between_robots > MIN_ROBOTS_DIST) && turtle_front_stop) {
    velCmd2Turtle.linear.y  = velocity;
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = false;
  } else {
    velCmd2Turtle.linear.y  = 0;
    velCmd2TurtleBack.publish(velCmd2Turtle);
    turtle_back_stop = true;
  }
}

// тек оценка маркера относительно 0 точки
void getCrntEstZrPntPose(const geometry_msgs::PoseStamped& arucoCameraMsg) { 
  estCrntCamZrPntPose = arucoCameraMsg;
  getPoseEstZrPnt     = true;
}

void go_vdrk_in_line(){

  // расстояние между маркером и 0 точкой
  distance_between_robots = std::sqrt(std::pow(estCrntCamZrPntPose.pose.position.x, 2) + 
                                      std::pow(estCrntCamZrPntPose.pose.position.y, 2) + 
                                      std::pow(estCrntCamZrPntPose.pose.position.z, 2));
  ROS_INFO("\ndistance_between_robots = %.5f\n", distance_between_robots);
  go_turtle_front();
  go_turtle_back();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "vdrk_move_line");
  ros::NodeHandle node;
  ros::param::param<double>("~Velocity", velocity, 0);

  setup();

  ros::Subscriber crntEstZrPntPoseSub =                                             
    node.subscribe("/aruco_single/pose", 0, getCrntEstZrPntPose);   // текущее оценочное положение относительно 0 точки

  velCmd2TurtleBack =
    node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);      // скорость для заднего робота

  velCmd2TurtleFront =
    node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel", 0);       // скорость для переднего робота

  ros::Rate loop_rate(30);

  while (ros::ok()) {

    ros::spinOnce();

    if (!getPoseEstZrPnt) continue;
    getPoseEstZrPnt = false;

    go_vdrk_in_line();

    loop_rate.sleep();
  }

  return 0;
}