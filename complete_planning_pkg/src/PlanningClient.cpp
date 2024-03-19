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
    actionlib::SimpleActionClient<complete_planning_msgs::SlerpPlanAction> slerp_plan_client("slerp_plan_action", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for JointPlan action server to start...");
    joint_plan_client.waitForServer();
    ROS_INFO("Waiting for SlerpPlan action server to start...");
    slerp_plan_client.waitForServer();

    // Create a goal to send to the action server
    complete_planning_msgs::JointPlanGoal goal;
    complete_planning_msgs::SlerpPlanGoal slerp_goal;
    // Load goal configuration from the parameter server
    std::vector<double> goal_pose_vec;
    if (!ros::param::get("/slerp_test_client/goal_configuration", goal_pose_vec))
    {
        ROS_ERROR("Failed to load goal_configuration parameter.");
        return 1;
    }
    slerp_goal.goal_pose.position.x = goal_pose_vec.at(0);
    slerp_goal.goal_pose.position.y = goal_pose_vec.at(1);
    slerp_goal.goal_pose.position.z = goal_pose_vec.at(2);

    slerp_goal.goal_pose.orientation.x = goal_pose_vec.at(3);
    slerp_goal.goal_pose.orientation.y = goal_pose_vec.at(4);
    slerp_goal.goal_pose.orientation.z = goal_pose_vec.at(5);
    slerp_goal.goal_pose.orientation.w = goal_pose_vec.at(6);

    // Load initial configuration from the parameter server
    if (!ros::param::get("/slerp_test_client/initial_configuration", slerp_goal.initial_configuration))
    {
        ROS_ERROR("Failed to load initial_configuration parameter.");
        return 1;
    }

    // Load planning group from the parameter server
    if (!ros::param::get("/slerp_test_client/planning_group", slerp_goal.planning_group))
    {
        ROS_ERROR("Failed to load planning_group parameter.");
        return 1;
    }
    std::vector<double> initial_pose_vec;
    if (!ros::param::get("/slerp_test_client/initial_pose", initial_pose_vec))
    {
        ROS_ERROR("Failed to load goal_configuration parameter.");
        return 1;
    }

    slerp_goal.initial_pose.position.x = initial_pose_vec.at(0);
    slerp_goal.initial_pose.position.y = initial_pose_vec.at(1);
    slerp_goal.initial_pose.position.z = initial_pose_vec.at(2);

    slerp_goal.initial_pose.orientation.x = initial_pose_vec.at(3);
    slerp_goal.initial_pose.orientation.y = initial_pose_vec.at(4);
    slerp_goal.initial_pose.orientation.z = initial_pose_vec.at(5);
    slerp_goal.initial_pose.orientation.w = initial_pose_vec.at(6);

    // Send the goal to the action server
    ROS_INFO("Sending goal to SlerpPlan action server...");
    slerp_plan_client.sendGoal(slerp_goal);

    // Wait for the action to complete (you can add a timeout here)
    bool finished_before_timeout = joint_plan_client.waitForResult();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = slerp_plan_client.getState();
        ROS_INFO("SlerpPlan action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_ERROR("SlerpPlan action did not complete before the timeout.");
    }
    ros::waitForShutdown();

    return 0;
}