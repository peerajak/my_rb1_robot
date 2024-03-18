#include "my_rb1_ros/Rotate.h"
#include "ros/ros.h"
// Import the service message used by the service /trajectory_by_name

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "my_rb1_service_client"); // Initialise a ROS node with the name
                                      // service_client
  ros::NodeHandle nh;

  // Create the connection to the service /trajectory_by_name
  ros::service::waitForService(
      "/rotate_robot"); // wait for service to be running
  ros::ServiceClient moverb1_service =
      nh.serviceClient<my_rb1_ros::Rotate>("/rotate_robot");
  my_rb1_ros::Rotate srv; // Create an object of type TrajByName
  srv.request.degrees = 12;

  if (moverb1_service.call(srv)) // Send through the connection the name of
                                 // the trajectory to execute
  {
    ROS_INFO("Umm"); // Print the result given by the service called
  } else {
    ROS_ERROR("Failed to call service /trajectory_by_name");
    return 1;
  }

  return 0;
}