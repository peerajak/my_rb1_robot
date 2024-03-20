#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

// Import the service message header file generated from the Empty.srv message
double yaw_rad = 0., pitch_rad = 0., roll_rad = 0., req_yaw_rad = 0.,
       target_yaw_rad = 0.;
bool rotate_complete = false;
geometry_msgs::Twist ling;
ros::Publisher pub;
bool startstop = true;

// We define the callback function of the service
bool my_callback(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Request Data==> degree=%d", req.degrees);
  req_yaw_rad = -req.degrees * 3.14 / 180;
  rotate_complete = false;
  res.result = "Success";
  return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  /*ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,
           msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,
           msg->twist.twist.angular.z);*/

  tf::Quaternion odom_quat(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 matrix_tf(odom_quat);
  matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
}

void setStopMovement() {
  ling.linear.x = 0;
  ling.linear.y = 0;
  ling.linear.z = 0;
  ling.angular.x = 0;
  ling.angular.y = 0;
  ling.angular.z = 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_rb1_service_server");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);
  ros::ServiceServer my_service = nh.advertiseService(
      "/rotate_robot",
      my_callback); // create the Service called // my_service
                    // with the defined // callback
  ros::Rate loop_rate(2);

  while (ros::ok()) {
    target_yaw_rad = req_yaw_rad + yaw_rad;
    ling.linear.x = 0;
    ling.linear.y = 0;
    ling.linear.z = 0;
    ling.angular.x = 0;
    ling.angular.y = 0;
    ling.angular.z = -0.5 * target_yaw_rad;

    pub.publish(ling);
    ROS_INFO("request=%f target=%f current:%f", req_yaw_rad, target_yaw_rad,
             yaw_rad);
    ros::spinOnce();
    setStopMovement();
    rotate_complete = true;
    loop_rate.sleep();
  }

  return 0;
}