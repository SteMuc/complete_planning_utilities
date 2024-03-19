/**
 * @file PlanningClient.cpp
 * @brief Implementation of a ROS node that start action clients for Cartesian planning.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "complete_planning_pkg/ActionClient.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_client_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // // Create instances of your action server classes
    // CartesianPlanActionClient cartesian_plan_client(nh, "cartesian_plan_action");

    // // Initialize arguments for the client
    // geometry_msgs::Pose goal_pose;
    // goal_pose.position.x = 0.5;
    // goal_pose.position.y = 0.0;
    // goal_pose.position.z = 0.2;

    // goal_pose.orientation.x = 1.0;
    // goal_pose.orientation.y = 0.0;
    // goal_pose.orientation.z = 0.0;
    // goal_pose.orientation.w = 0.0;

    // // std::vector<double> initial_configuration{1.3, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483};
    // std::vector<double> initial_configuration;

    // std::string planning_group("panda_arm");

    // // Send the goal to the action server
    // ROS_INFO("Sending goal to CartesianPlan action server...");

    // GoalHandle goal_handle1 = cartesian_plan_client.sendGoal(goal_pose, initial_configuration, planning_group);
    // //                           // Spin to keep the node alive
    // cartesian_plan_client._goalHandles[1] = &goal_handle1;
    // // while (cartesian_plan_client._goalHandles[1]->getCommState().toString() != "DONE")
    // // {
    // //     ROS_INFO("Not finished yet");
    // // }
    // Create an Action client for the JointPlan action server
    actionlib::SimpleActionClient<complete_planning_msgs::JointPlanAction> joint_plan_client("joint_plan_action", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for JointPlan action server to start...");
    joint_plan_client.waitForServer();

    // Create a goal to send to the action server
    complete_planning_msgs::JointPlanGoal goal;
    // Load goal configuration from the parameter server
    if (!ros::param::get("/joint_test_client/goal_configuration", goal.goal_configuration))
    {
        ROS_ERROR("Failed to load goal_configuration parameter.");
        return 1;
    }

    // Load initial configuration from the parameter server
    if (!ros::param::get("/joint_test_client/initial_configuration", goal.initial_configuration))
    {
        ROS_ERROR("Failed to load initial_configuration parameter.");
        return 1;
    }

    // Load planning group from the parameter server
    if (!ros::param::get("/joint_test_client/planning_group", goal.planning_group))
    {
        ROS_ERROR("Failed to load planning_group parameter.");
        return 1;
    }
    // Send the goal to the action server
    ROS_INFO("Sending goal to JointPlan action server...");
    joint_plan_client.sendGoal(goal);

    // Wait for the action to complete (you can add a timeout here)
    bool finished_before_timeout = joint_plan_client.waitForResult();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = joint_plan_client.getState();
        ROS_INFO("JointPlan action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_ERROR("JointPlan action did not complete before the timeout.");
    }
    ros::waitForShutdown();

    return 0;
}