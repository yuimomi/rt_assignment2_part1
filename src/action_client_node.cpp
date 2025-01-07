#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <my_assignment2/PositionVelocity.h>
#include <string>
#include <sstream>

ros::Publisher robot_state_pub;
ros::Publisher cmd_vel_pub; 

my_assignment2::PositionVelocity robot_state;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_state.x = msg->pose.pose.position.x;
    robot_state.y = msg->pose.pose.position.y;
    robot_state.vel_x = msg->twist.twist.linear.x;
    robot_state.vel_z = msg->twist.twist.angular.z;

    // Publish the robot status
    robot_state_pub.publish(robot_state);
}

void stopRobot() {
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub.publish(stop_msg);
    ROS_INFO("Robot stopped.");
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_client_node");
  ros::NodeHandle nh;

  // Initialize subscriber and publisher
  robot_state_pub = nh.advertise<my_assignment2::PositionVelocity>("/robot_state", 10);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 


  // create the action client
  actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started.");

  while (ros::ok()) {
      std::string input;
      ROS_INFO("Enter target coordinates (x y) or type 'cancel' to cancel the goal:");
      std::getline(std::cin, input);

      // Check if the user wants to cancel the goal
      // if (input == "cancel") {
      //     ROS_INFO("Cancelling the current goal...");
      //     ac.cancelGoal();
      //     ros::Duration(1.0).sleep(); // Wait briefly for the cancellation to take effect
      //     continue;
      // }
        if (input == "cancel") {
            auto state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE || 
                state == actionlib::SimpleClientGoalState::PENDING) {
                ROS_INFO("Cancelling the current goal...");
                ac.cancelGoal();
                stopRobot();      
                ros::Duration(1.0).sleep(); // Wait briefly for the cancellation to take effect
            } else {
                ROS_WARN("No active goal to cancel.");
            }
            continue;
        }     

      // Parse the input for target coordinates
      std::istringstream iss(input);
      double target_x, target_y;
      if (!(iss >> target_x >> target_y)) {
          ROS_WARN("Invalid input. Please enter coordinates as two numbers (x y) or 'cancel'.");
          continue;
      }

  // get the target coordinates from the user
  // double target_x, target_y;
  // ROS_INFO("Enter target coordinates (x, y): ");
  // std::cin >> target_x >> target_y;

  // send a goal to the action
  assignment_2_2024::PlanningGoal goal;
  goal.target_pose.pose.position.x = target_x;
  goal.target_pose.pose.position.y = target_y;

  ROS_INFO("Sending goal...");
  ac.sendGoal(goal);

    // Wait for the result or user cancellation
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
            stopRobot(); 
            break;
        }

        // During the robot is moving toward goal, it sends feedbacks
        // ROS_INFO("Moving towards goal...");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
  }
}
