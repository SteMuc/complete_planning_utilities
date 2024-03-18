/**
 * @file PlanningClient.cpp
 * @brief Implementation of a ROS node that start action clients for Cartesian planning.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <ros/ros.h>
#include "complete_planning_pkg/ActionClient.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_client_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create instances of your action server classes
    CartesianPlanActionClient cartesian_plan_client(nh, "cartesian_plan_action");

    // Initialize arguments for the client
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = 0.5;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.2;

    goal_pose.orientation.x = 1.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;

    std::vector<double> initial_configuration{1.3, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483};

    std::string planning_group("panda_arm");


    // Send the goal to the action server
    ROS_INFO("Sending goal to CartesianPlan action server...");

    GoalHandle goal_handle1 = cartesian_plan_client.sendGoal(goal_pose, initial_configuration, planning_group);
    //                           // Spin to keep the node alive
    cartesian_plan_client._goalHandles[1] = &goal_handle1;
    while(cartesian_plan_client._goalHandles[1]->getCommState().toString() != "DONE"){
          ROS_INFO("Not finished yet");
    }
    ROS_INFO("DONE: Before ros::waitForShutDown");
    ros::waitForShutdown(); 

    return 0;
}