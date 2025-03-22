# rt_assignment2_part1
---

## Summary
This package provides an example of implementing a task with an action client in ROS. It focuses on enabling the robot to navigate to a target location without blocking other processes. The implementation is distributed across two nodes and a launch file, as outlined below:


## Node Overview

 **Action Client Node (`action_client_node`)**
- **Description**: This node manages communication with the action server for robot navigation tasks.
- **Features**:
  - Sends target coordinates `(x, y)` to the action server.
  - Allows the user to cancel the goal during execution.
  - Tracks feedback and status from the action server to determine goal completion.
  - Publishes the robot's position and velocity as a custom message (`x`, `y`, `vel_x`, `vel_z`) using data from the `/odom` topic.
 
- **Subscriber**
  - Topic: `/odom`
    Receives the robot's odometry data.
  - Message Type: `nav_msgs/Odometry`

- **Publisher**
  - Topic: `/robot_state`
    Publishes the robot's position (x, y) and velocity (vel_x, vel_z) in a simplified custom format.
  - Message Type: `my_assignment2/PositionVelocity`
---

**Service Node (`target_service_node`)**
- **Description**: This node provides information about the most recent goal sent by the user.
- **Features**:
  - Responds to service requests with the coordinates of the last goal.
 

## Launch File
**Launch Configuration**
- Starts the simulation environment and initializes the following nodes:
  - `action_client_node`
  - `target_service_node`

---

## Usage

**Step 1: Build the Package**

First, make sure you have assignment_2_2024 in your workspace.

You can clone it from github with this link :

https://github.com/CarmineD8/assignment_2_2024

Then, clone the rt_assignment2_part1 repository into your ros workspace and compile with:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Step 2: Launch the Nodes**

First, launch the simlation environment with:
```bash
roslaunch assignment_2_2024 assignment1.launch 
```
Then, launch action client node and service node.
```bash
roslaunch my_assignment2 my_assignment2.launch
```

**Step 3: Interact with the System**
Once the nodes launched, you have three choices with the robot_target node

1. Enter a target coordinate
   ```bash
   x y
   ```
  for example
   ```bash
   2.0 3.0
   ```
2. Cancel a target coordinate and stop the robot
```bash
cancel
```

## Node Description
## action_client_node
This node is composed of two main components:

**1. Action Client with Target Control**
- Interacts with the action server `reaching_goal` using the `PlanningAction` message from the `assignment_2_2024` package.
- Runs in a separate thread to allow:
  - Sending new target coordinates.
  - Canceling an active goal at any time.
- Provides real-time control over the robot's navigation tasks while maintaining seamless communication with the action server.

**2. Subscriber-Publisher for Odometry Simplification**
- Subscribes to the `/odom` topic to receive the robot's odometry data.
- Processes the data to extract simplified position and velocity information.
- Publishes this simplified data to the `/robot_state` topic using the custom message format `PositionVelocity`.

**Subscriber**
- **Topic**: `/odom`
  - **Receives**: Robot's odometry data.
  - **Message Type**: `nav_msgs/Odometry`

**Publisher**
- **Topic**: `/robot_state`
  - **Publishes**: Simplified position (`x`, `y`) and velocity (`vel_x`, `vel_z`).
  - **Message Type**: `my_assignment2/PositionVelocity`


## target_service_node
The `target_service_node` tracks the last target coordinates received by the robot. It subscribes to `/last_target_topic` to receive `geometry_msgs/Point` messages and stores the coordinates in `last_x` and `last_y`. When the `/get_last_target` service is called, it returns the stored coordinates.


**`/get_last_target`**
- **Description**: Provides the most recent target position that was sent to the robot.
- **Request Type**: `my_assignment2/LastTarget::Request`
- **Response Type**: `my_assignment2/LastTarget::Response`
  - **Fields**:
    - `x`: X-coordinate of the last target.
    - `y`: Y-coordinate of the last target.


**Subscriber**
- **Topic**: `/last_target_topic`
  - **Description**: Receives target coordinates sent to the robot.
  - **Message Type**: `geometry_msgs/Point`

