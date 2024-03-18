#include "my_rb1_ros/Rotate.h"
#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Twist.h>
// Import the service message header file generated from the Empty.srv message
geometry_msgs::Twist ling;
ros::Publisher pub;
bool startstop = true;
void setCircularMovement() {
  ling.linear.x = 2;
  ling.linear.y = 0;
  ling.linear.z = 0;
  ling.angular.x = 0;
  ling.angular.y = 0;
  ling.angular.z = 2;
}
void setStopMovement() {
  ling.linear.x = 0;
  ling.linear.y = 0;
  ling.linear.z = 0;
  ling.angular.x = 0;
  ling.angular.y = 0;
  ling.angular.z = 0;
}

// We define the callback function of the service
bool my_callback(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Request Data==> degree=%d", req.degrees);
  if (req.degrees > 5) {
    res.result = "success";
    ROS_INFO("sending back response:true");
  } else {
    res.result = "failed";
    ROS_INFO("sending back response:false");
  }
  int i = 0;
  setCircularMovement();
  while (i < req.degrees) {
    pub.publish(ling);
    usleep(1000000); // We set 1000000 because the time is set in microseconds
    i++;
  }
  setStopMovement();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_rb1_service_server");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::ServiceServer my_service = nh.advertiseService(
      "/rotate_robot",
      my_callback); // create the Service called // my_service
                    // with the defined // callback

  ros::Rate loop_rate(2);

  while (ros::ok()) {
    pub.publish(ling);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}