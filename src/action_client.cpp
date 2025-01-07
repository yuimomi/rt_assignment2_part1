#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/PositionVelocity.h>

ros::Publisher robot_state_pub;

custom_msgs::PositionVelocity robot_state;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_state.x = msg->pose.pose.position.x;
    robot_state.y = msg->pose.pose.position.y;
    robot_state.vel_x = msg->twist.twist.linear.x;
    robot_state.vel_z = msg->twist.twist.angular.z;

    // Publish the robot status
    robot_state_pub.publish(robot_state);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_client");
  ros::NodeHandle nh;

  // Initialize subscriber and publisher
  robot_state_pub = nh.advertise<custom_msgs::PositionVelocity>("/robot_state", 10);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

  // create the action client
  actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started.");

  // get the target coordinates from the user
  double target_x, target_y;
  ROS_INFO("Enter target coordinates (x, y): ");
  std::cin >> target_x >> target_y;

  // send a goal to the action
  assignment_2_2024::PlanningGoal goal;
  goal.target_pose.pose.position.x = target_x;
  goal.target_pose.pose.position.y = target_y;

  ROS_INFO("Sending goal...");
  ac.sendGoal(goal);

  // loop for recieving feedback
    ros::Rate rate(10);
    while (ros::ok()) {
        auto state = ac.getState();
        ROS_INFO("Action State: %s", state.toString().c_str());

        // if the robot reached the target
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Target reached!");
            break;
        }

        // if the goal is canceled
        if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_WARN("Target canceled by user.");
            break;
        }

        // During the robot is moving toward goal, it sends feedbacks
        ROS_INFO("Moving towards goal...");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

  //wait for the action to return
//  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  //if (finished_before_timeout)
  //{
    //actionlib::SimpleClientGoalState state = ac.getState();
    //ROS_INFO("Action finished: %s",state.toString().c_str());
  //}
  //else
    //ROS_INFO("Action did not finish before the time out.");

  //exit
  //return 0;
