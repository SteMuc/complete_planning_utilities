Initial version of the planning_utilities for controlling robotic manipulators at Research Center "Enrico Piaggio" using MoveIt.

# complete_planning_utilities - ROS Metapackage

The `complete_planning_utilities` metapackage is a comprehensive suite of tools and ROS nodes designed to streamline robotic path planning and execution tasks. It offers various planning actions and provides a convenient way to manage and execute tasks involving Cartesian planning, joint planning, and plan execution.

## Overview

In the field of robotics, path planning and execution are essential for enabling robots to perform tasks accurately and safely. The `complete_planning_utilities` metapackage simplifies these tasks by providing a collection of tools and ROS nodes that facilitate the entire process.

## Packages

### 1. `complete_planning_pkg`

The `complete_planning_pkg` package, a crucial component of the `complete_planning_utilities` metapackage, contains several ROS action servers and task execution functionalities. Here are the key functionalities offered by the `complete_planning_pkg` package:

#### a. Action Servers
   - **`CartesianPlanActionServer`**: This class serves as a base for a Cartesian Plan Action Server. It handles Cartesian path planning goals, manages trajectories, and plans them for specified planning groups.

   - **`CartesianPlanDisplacementActionServer`**: This class serves as a base for a Cartesian Plan Displacement Action Server. It handles Cartesian displacement path planning goals, manages trajectories, and plans them for specified planning groups.

   - **`SlerpPlanActionServer`**: This class serves as a base for a SLERP Plan Action Server. It handles [Slerp](https://it.wikipedia.org/wiki/Slerp) path planning goals between end-effector and goal pose to create homogeneous motion.

   - **`SlerpPlanDisplacementActionServer`**: This class serves as a base for a SLERP Plan Displacement Action Server. It handles [Slerp](https://it.wikipedia.org/wiki/Slerp) displacement path planning goals between end-effector and goal pose to create homogeneous motion.

   - **`JointPlanActionServer`**: Similar to `CartesianPlanActionServer`, this class serves as a base for a Joint Plan Action Server. It handles joint path planning goals, manages joint configurations, and plans them for specified planning groups.

   - **`ExecutePlanActionServer`**: This class enables the execution of planned trajectories using an action server. It manages the execution of motion plans and provides callback functions for goal completion.

#### b. Task Handler
   - The `TaskHandler` class is designed to handle task execution using ROS action clients. It provides functionalities to plan and execute tasks based on specified goals. Tasks can involve both Cartesian and joint planning goals.

### 2. `complete_planning_msgs`

The `complete_planning_msgs` package defines custom ROS action and message types used by the action servers and other components in the `complete_planning_utilities` metapackage. These message types facilitate the communication of planning goals and results between nodes.

## Usage

To use the `complete_planning_utilities` metapackage in your ROS environment, follow these steps:

1. **Clone the Package**: Clone the `complete_planning_utilities` package to your ROS workspace.

2. **Build the Package**: Use `catkin_make` to build the package within your workspace.

3. **Define Tasks**: Define tasks and goals in YAML format, as shown in the example below. These tasks can include Cartesian and joint planning goals and options for merging or not merging planned configurations.

4. **Launch the Nodes**: Use the provided launch files to start the ROS nodes. The launch file loads task parameters from a YAML file and launches the action server and client nodes responsible for task planning and execution.



## Creating a YAML Task File

To plan and execute a sequence of tasks using the `task_planner_node`, you need to create a YAML task file. Each task in the file represents a specific motion planning goal, which can be either Cartesian planning or joint planning. 
Each task entry contains the following components:

1. **type**: Specifies the type of the planning task (one belong to the list specified above)

2. **goal**: Depending on the task type, define the planning goal, including the desired position and orientation for Cartesian planning or a list of joint angles for joint planning.

3. **merge**: Indicates whether to use as initial condition for planning the current configuration of the final configuration planned by the previous trajectory.

4. **group**: Specifies the planning group associated with the task.

You can add more tasks to the YAML file to create a sequence of motion planning tasks to be executed in the order they appear.
Below is a guide on how to build a YAML task file:

```yaml
tasks:
  - type: "CartesianPlan"
    goal:
      position:
        x: 0.4
        y: 0.1
        z: 0.4
      orientation:
        x: 0.0
        y: 1.0
        z: 0.0
        w: 0.0
    merge: false
    group: "panda_arm"

  - type: "JointPlan"
    goal: [0.17487751478777983, -1.2211202568068131, 0.41834634694816447, -2.4381058886000555, 0.4108761315889708, 1.2724048530962113, 1.238913433831517]
    merge: false
    group: "panda_arm"

  - type: "SlerpPlan"
    goal:
      position:
        x: 0.4
        y: 0.205
        z: 0.5
      orientation:
        x: 0.924
        y: -0.383
        z: 0.0
        w: 0.0
    number_of_waypoints: 10 
    merge: false
    group: "panda_arm"

  - type: "CartesianDisplacementPlan"
    goal:
      position_displacement:
        x: 0.1
        y: 0.0
        z: 0.0
      angular_displacement:
        x: -0.2 # Roll 
        y:  0.0 # Pitch
        z:  0.0 # Yaw
    merge: true
    group: "panda_arm"

  - type: "SlerpDisplacementPlan"
    goal:
      position_displacement:
        x: 0.2
        y: 0.0
        z: 0.0
      angular_displacement:
        x:  0.0 # Roll 
        y:  0.0 # Pitch
        z:  0.2 # Yaw
    number_of_waypoints: 10
    merge: true
    group: "panda_arm"

# Continue with more tasks...
```
