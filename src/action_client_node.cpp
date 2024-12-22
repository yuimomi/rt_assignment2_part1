#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// SSgo assignment_2_2024 k MoveBaseAction(~_oìêAction) LBhî
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_msgs/Odometry.h>
#include "my_assignment2/PositionVelocity.h"

class MyActionClient
{
public:
  MyActionClient()
  : ac_("move_base", true)  // : move_base ActionServeró
  {
    // Publishern»ÃÈ¢Ã×
    pub_ = nh_.advertise<my_assignment2::PositionVelocity>("pos_vel", 10);

    // Odomü­-
    sub_ = nh_.subscribe("/odom", 10, &MyActionClient::odomCallback, this);

    // µüÐ¥a
    ROS_INFO("Waiting for action server to start...");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    // æü¶e(n¹ìÃÉjiËa
RfD
  }

  void sendGoal(double x, double y)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // : map§û
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal: x=%f, y=%f", x, y);

    // ´üëá (Õ£üÉÐÃ¯(³üëÐÃ¯{2ï)
    ac_.sendGoal(goal,
                 boost::bind(&MyActionClient::doneCb, this, _1, _2),
                 boost::bind(&MyActionClient::activeCb, this),
                 boost::bind(&MyActionClient::feedbackCb, this, _1));
  }

  void cancelGoal()
  {
    ROS_INFO("Cancelling goal...");
    ac_.cancelGoal();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  // SimpleActionClient
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

  // íÜÃÈ¶KÝY	p
  double robot_x_, robot_y_, robot_vel_x_, robot_vel_z_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // OdomKMnh¦Ö
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    // íÜÃÈn&2¦hÞâ¦(!ktwistgÖ)
    robot_vel_x_ = msg->twist.twist.linear.x;
    robot_vel_z_ = msg->twist.twist.angular.z;

    // «¹¿àáÃ»ü¸kpfÑÖêÃ·å
    my_assignment2::PositionVelocity pv;
    pv.x = robot_x_;
    pv.y = robot_y_;
    pv.vel_x = robot_vel_x_;
    pv.vel_z = robot_vel_z_;
    pub_.publish(pv);
  }

  // ¢¯·çó¢#n³üëÐÃ¯
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    ROS_INFO("Goal finished with state: %s", state.toString().c_str());
  }

  void activeCb()
  {
    ROS_INFO("Goal just went active...");
  }

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    // Õ£üÉÐÃ¯h: (Åj)
    // ROS_INFO("Feedback: current pose: (%f, %f)",
    //           feedback->base_position.pose.position.x,
    //           feedback->base_position.pose.position.y);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_action_client_node");

  MyActionClient client;

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // hWfæü¶eÝüêó°~_oµüÓ¹/ÈÔÃ¯×ágþÜ
    // SSgoXk´üëá/­ãó»ëY
    // ochÔßkÅY

    // e.g., sendGoal(1.0, 2.0);
    // e.g., cancelGoal();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
