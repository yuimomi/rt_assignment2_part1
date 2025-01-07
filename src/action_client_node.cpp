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

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_state.x = msg->pose.pose.position.x;
    robot_state.y = msg->pose.pose.position.y;
    robot_state.vel_x = msg->twist.twist.linear.x;
    robot_state.vel_z = msg->twist.twist.angular.z;

    // Publish the robot status
    robot_state_pub.publish(robot_state);
}

// function to stop robot
void stopRobot() {
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub.publish(stop_msg);
    ROS_INFO("Robot stopped.");
}

// thread function to process user input
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
