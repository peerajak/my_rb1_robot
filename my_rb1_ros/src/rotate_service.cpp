#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <tf/tf.h>

// Import the service message header file generated from the Empty.srv message
double yaw_rad = 0., pitch_rad = 0., roll_rad = 0., req_yaw_rad = 0.,
       target_yaw_rad = 0.;
bool rotate_complete = false;
geometry_msgs::Twist ling;
ros::Publisher pub;
bool startstop = true;
const double pi = 3.14;

struct Radian {
  double _rad, _additional_rad; // range 0 to 2pi
  double const pi = 3.14;
  Radian(double rad) : _additional_rad(0) {
    if (rad < 0) {
      int nearest_greater_mulitple_of_round = (-1 * rad) / (2 * pi) + 1;
      _rad = rad + double(nearest_greater_mulitple_of_round) * 2 * pi;

    } else {
      int nearest_smaller_mulitple_of_round = (1 * rad) / (2 * pi);
      _rad = rad - double(nearest_smaller_mulitple_of_round) * 2 * pi;
    }
  }

  double get_position() { return _rad + _additional_rad; }

  double operator-(Radian const &obj) {
    _additional_rad = -obj._rad;
    return get_position();
  }

  double operator+(Radian const &obj) {
    _additional_rad = obj._rad;
    return get_position();
  }
};

struct NRadian {
  double _rad; // range -0 to -2pi
  NRadian(double rad) {
    if (rad > 0) {
      int nearest_greater_mulitple_of_round = (-1 * rad) / (2 * pi) - 1;
      _rad = rad + double(nearest_greater_mulitple_of_round * 2 * pi);

    } else {
      int nearest_smaller_mulitple_of_round = (1 * rad) / (2 * pi);
      _rad = rad - double(nearest_smaller_mulitple_of_round) * 2 * pi;
    }
  }
};

// We define the callback function of the service
bool my_callback(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Request Data==> degree=%d", req.degrees);
  ros::Rate loop_rate(2);
  const double threshold = 0.001;
  if (req.degrees > 360 || req.degrees < -360) {
    res.result = "Service failed: The range of degree is between -360 to 360";
  } else {

    Radian R_yaw_rad(yaw_rad);
    Radian R_raw_req_yaw_rad(req.degrees * pi / 180);
    req_yaw_rad = R_raw_req_yaw_rad + R_yaw_rad;
    Radian R_req_yaw_rad(req_yaw_rad);

    unsigned int timeout_counter = 0, max_timeout_counter = 50;
    double target_yaw_rad_temp;
    do {
      Radian R_yaw_rad_current(yaw_rad);
      if (req.degrees > 0) {
        if (R_yaw_rad_current._rad > R_req_yaw_rad._rad) {
          ROS_DEBUG("Add 2pi to req");
          R_req_yaw_rad._rad += 2 * pi;
        }
        target_yaw_rad = R_req_yaw_rad - R_yaw_rad_current;
        Radian R_target_yaw_rad(target_yaw_rad);
        target_yaw_rad_temp = R_target_yaw_rad._rad;
      } else {
        if (R_yaw_rad_current._rad < R_req_yaw_rad._rad) {
          ROS_DEBUG("Remove 2pi from Req");
          R_req_yaw_rad._rad -= 2 * pi;
        }
        target_yaw_rad = R_req_yaw_rad - R_yaw_rad_current;
        NRadian R_target_yaw_rad(target_yaw_rad);
        target_yaw_rad_temp = R_target_yaw_rad._rad;
      }

      ling.linear.x = 0;
      ling.linear.y = 0;
      ling.linear.z = 0;
      ling.angular.x = 0;
      ling.angular.y = 0;
      ling.angular.z = 0.5 * target_yaw_rad_temp;

      pub.publish(ling);
      ROS_DEBUG("request=%f.%f target=%f current:%f.%f", R_req_yaw_rad._rad,
                R_req_yaw_rad._additional_rad, target_yaw_rad_temp,
                R_yaw_rad_current._rad, R_yaw_rad_current._additional_rad);
      ros::spinOnce();
      loop_rate.sleep();
      timeout_counter++;

    } while (abs(target_yaw_rad_temp) > threshold and
             timeout_counter <= max_timeout_counter);
    res.result = abs(target_yaw_rad_temp) <= threshold ? "Service Success"
                                                       : "Service Failed";
  }
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

void mySigintHandler(int sig) {
  ROS_INFO("/rotate_robot service stopped");
  ros::shutdown();
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
  signal(SIGINT, mySigintHandler);
  ROS_INFO("/rotate_robot service started");
  ros::spin();
  // while (ros::ok()) {
  // ROS_INFO("request=%f target=%f current:%f", req_yaw_rad, target_yaw_rad,
  // yaw_rad);
  // ros::spinOnce();
  // setStopMovement();
  // rotate_complete = true;
  // loop_rate.sleep();
  //}
  ROS_INFO("/rotate_robot service stopped");

  return 0;
}