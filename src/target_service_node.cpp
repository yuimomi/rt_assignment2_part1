/**
 * \file target_service_node.cpp
 * \brief Service client node for assignment 2
 * \author Yui Momiyama
 * \version 1.0
 * \date 21/03/2025

 * \details
 *
 * Subscribes to: <BR>
 * /last_target_topic
 * 
 * Services: <BR>
 * get_last_target
 *
 * Description :
 *
 * This node is a service client that listens to the "/last_target_topic" topic 
 * and provides a service called "get_last_target" that returns the last target that was published to the topic. 
 * The service returns the x and y coordinates of the last target as a response.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h> 
#include "my_assignment2/LastTarget.h"

double last_x = 0.0; ///< x coordinate of the last target
double last_y = 0.0; ///< y coordinate of the last target

/**
 * \brief Callback function for the "/last_target_topic" topic
 * \param msg A pointer to the recieved geometry_msgs::Point message
 *
 * This function is a callback function that is called whenever a message is published.
 * It saves the x and y coordinates of the last target that was published to the topic.
 */

void targetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  last_x = msg->x;
  last_y = msg->y;
}

/**
 * \brief Callback function for the service "get_last_target".
 * \param req A pointer to the recieved my_assignment2::LastTarget::Request message
 * \param res A pointer to the recieved my_assignment2::LastTarget::Response message
 * 
 * \return always true as this method cannot fail
 * 
 * This function is a callback function that is called whenever the service is called.
 * It returns the x and y coordinates of the last target as a response.
 */

bool getLastTarget(my_assignment2::LastTarget::Request &req,
                   my_assignment2::LastTarget::Response &res)
{
  res.x = last_x;
  res.y = last_y;
  ROS_INFO("Service called. Returning last target: (%f, %f)", last_x, last_y);
  return true;
}

 /**
  * \brief Main function for the target_service_node.
  * 
  * \param argc Number of input arguments (if any)
  * \param argv Pointer to array of input arguments (if any)
  * 
  * \return 0 if the program exits successfully
  * 
  * Initializes the service node and creates a service server.
  * Then, it subscribes to the "/last_target_topic" topic.
  * Finally, it keeps the node running until it is stopped.
  */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_service_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);

  ros::Subscriber sub = nh.subscribe("/last_target_topic", 1000, targetCallback);

  ROS_INFO("LastTargetService is ready.");
  ros::spin();
  return 0;
}
