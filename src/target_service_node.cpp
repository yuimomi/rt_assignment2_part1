#include <ros/ros.h>
#include "my_assignment2/LastTarget.h"

double last_x = 0.0;
double last_y = 0.0;

void targetCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  last_x = msg->x;
  last_y = msg->y;
}

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

  ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);

  ros::Subscriber sub = nh.subscribe("/last_target_topic", 10, targetCallback);

  ros::spin();
  return 0;
}
