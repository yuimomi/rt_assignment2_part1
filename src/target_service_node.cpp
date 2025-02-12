#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h> 
#include "my_assignment2/LastTarget.h"

double last_x = 0.0;
double last_y = 0.0;

// This function is called whenever a message is published to the "/last_target_topic" topic
void targetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  last_x = msg->x;
  last_y = msg->y;
}

// This function is called whenever the service is called
bool getLastTarget(my_assignment2::LastTarget::Request &req,
                   my_assignment2::LastTarget::Response &res)
{
  res.x = last_x;
  res.y = last_y;
  ROS_INFO("Service called. Returning last target: (%f, %f)", last_x, last_y);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_service_node");
  ros::NodeHandle nh;

  // Create a service server with the name "get_last_target" that uses the getLastTarget callback function
  ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);

  // Create a subscriber to the topic "/last_target_topic" that uses the targetCallback function
  ros::Subscriber sub = nh.subscribe("/last_target_topic", 1000, targetCallback);

  ROS_INFO("LastTargetService is ready.");
  ros::spin(); // Keep the node running until it is stopped
  return 0;
}
