/**
 * \file action_client_node.cpp
 * \brief Action client node for assignment 2
 * \author Yui Momiyama
 * \version 1.0
 * \date 21/03/2025

 * \details
 *
 * Subscribes to: <BR>
 * ° /odom
 *
 * Publishes to: <BR>
 * ° /robot_state
 *  ° /last_target
 * ° /cmd_vel
 *
 * Description :
 *
 * This node is an action client that sends goals to the action server to reach a target position.
 * The user can input the target position from the console.
 *
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/PlanningAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <my_assignment2/PositionVelocity.h>
#include <thread>
#include <atomic>
#include <string>
#include <sstream>

ros::Publisher robot_state_pub;
ros::Publisher cmd_vel_pub;  // publisher to stop the robot
std::atomic<bool> cancel_goal(false);  // flag to cancel goal

my_assignment2::PositionVelocity robot_state;

/**
 * \brief Callback function for the /odom topic.
 * \param msg A pointer to the recieved odometry
 *
 * This function is a callback function that is called when the robot's position and velocity is received.
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_state.x = msg->pose.pose.position.x;
    robot_state.y = msg->pose.pose.position.y;
    robot_state.vel_x = msg->twist.twist.linear.x;
    robot_state.vel_z = msg->twist.twist.angular.z;

    // Publish the robot status
    robot_state_pub.publish(robot_state);
}

/**
 * \brief Function to stop the robot.
 *
 * This function stops the robot by publishing a message with zero linear and angular velocity.
 */
void stopRobot() {
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub.publish(stop_msg);
    ROS_INFO("Robot stopped.");
}

/**
 * \brief Thread function to process user input from the console.
 * \param ac A pointer to the action client
 * 
 * This function manages user input from the console. 
 * The user can enter target coordinates to send a new goal or cancel the current goal.
 */

void userInputThread(actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>& ac) {
    while (ros::ok()) {
        std::string input;
        ROS_INFO("Enter target coordinates (x y) or type 'cancel' to cancel the goal:");
        std::getline(std::cin, input);

        if (input == "cancel") {
            auto state = ac.getState();
            if (state == actionlib::SimpleClientGoalState::ACTIVE || 
                state == actionlib::SimpleClientGoalState::PENDING) {
                ROS_INFO("Cancelling the current goal...");
                cancel_goal = true;  // set flag
                ac.cancelGoal();  // cancel goal
            } else {
                ROS_WARN("No active goal to cancel.");
            }
        } else {
            std::istringstream iss(input);
            double target_x, target_y;
            if (!(iss >> target_x >> target_y)) {
                ROS_WARN("Invalid input. Please enter coordinates as two numbers (x y) or 'cancel'.");
            } else {
                ROS_INFO("Sending new goal: x=%.2f, y=%.2f", target_x, target_y);
                assignment_2_2024::PlanningGoal goal;
                goal.target_pose.pose.position.x = target_x;
                goal.target_pose.pose.position.y = target_y;
                ac.sendGoal(goal);  // send goal
                cancel_goal = false;  // reset the flag
            }
        }
    }
}

 /**
  * \brief Main function for the action_client_node.
  * 
  * \param argc Number of input arguments (if any)
  * \param argv Pointer to array of input arguments (if any)
  * 
  * \return 0 if the program exits successfully
  * 
  * Initializes the action client node, subscribers, and publishers.
  * and creates the action client.
  * Then, it starts a thread to process user input and enters a loop to move the robot.
  */

int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_client_node");
    ros::NodeHandle nh;

    // Initialize subscribers and publishers
    robot_state_pub = nh.advertise<my_assignment2::PositionVelocity>("/robot_state", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  // ロボットを停止させるためのトピック

    // Create the action client
    actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("reaching_goal", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started.");

    // Start a thread to process user input
    std::thread input_thread(userInputThread, std::ref(ac));

    // loop for moving the robot
    ros::Rate rate(10);
    while (ros::ok()) {
        auto state = ac.getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Target reached!");
        } else if (state == actionlib::SimpleClientGoalState::PREEMPTED && cancel_goal) {
            ROS_WARN("Goal canceled by user.");
            stopRobot();  // stop robot
            cancel_goal = false;  // reset flag
        }

        ros::spinOnce();
        rate.sleep();
    }

    // finish thread
    input_thread.join();
    return 0;
}
