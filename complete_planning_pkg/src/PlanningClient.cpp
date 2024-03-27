/**
 * @file PlanningClient.cpp
 * @brief Implementation of a ROS node that start action clients for Cartesian planning.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include "complete_planning_pkg/ActionClient.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_client_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    actionlib::SimpleActionClient<complete_planning_msgs::JointPlanAction> joint_plan_client("joint_plan_action", true);
    actionlib::SimpleActionClient<complete_planning_msgs::SlerpPlanAction> slerp_plan_client("slerp_plan_action", true);
    actionlib::SimpleActionClient<complete_planning_msgs::SlerpPlanDisplacementAction> slerp_plan_displacement_client("slerp_plan_displacement_action", true);
    actionlib::SimpleActionClient<complete_planning_msgs::CartesianPlanAction> cartesian_plan_client("cartesian_plan_action", true);
    actionlib::SimpleActionClient<complete_planning_msgs::CartesianPlanDisplacementAction> cartesian_plan_displacement_client("cartesian_plan_displacement_action", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for JointPlan action server to start...");
    joint_plan_client.waitForServer();
    ROS_INFO("Waiting for SlerpPlan action server to start...");
    slerp_plan_client.waitForServer();
    ROS_INFO("Waiting for CartesianPlan action server to start...");
    cartesian_plan_client.waitForServer();
    ROS_INFO("Waiting for SlerpPlanDisplacement action server to start...");
    slerp_plan_displacement_client.waitForServer();
    ROS_INFO("Waiting for CartesianPlanDisplacement action server to start...");
    cartesian_plan_displacement_client.waitForServer();

    // Create a goal to send to the action server
    complete_planning_msgs::JointPlanGoal goal;
    complete_planning_msgs::SlerpPlanGoal slerp_goal;
    complete_planning_msgs::SlerpPlanDisplacementGoal slerp_displacement_goal;
    complete_planning_msgs::CartesianPlanDisplacementGoal cartesian_displacement_goal;
    complete_planning_msgs::CartesianPlanGoal cartesian_goal;

    // Load goal configuration from the parameter server
    std::vector<double> pos_disp_vec;
    if (!ros::param::get("/cartesian_displacement_test_client/pos_disp", pos_disp_vec))
    {
        ROS_ERROR("Failed to load pos_disp parameter.");
        return 1;
    }
    cartesian_displacement_goal.pos_disp.x = pos_disp_vec.at(0);
    cartesian_displacement_goal.pos_disp.y = pos_disp_vec.at(1);
    cartesian_displacement_goal.pos_disp.z = pos_disp_vec.at(2);

    std::vector<double> angular_disp_vec;
    if (!ros::param::get("/cartesian_displacement_test_client/angular_disp", angular_disp_vec))
    {
        ROS_ERROR("Failed to load angular_disp parameter.");
        return 1;
    }
    cartesian_displacement_goal.angular_disp.x = angular_disp_vec.at(0);
    cartesian_displacement_goal.angular_disp.y = angular_disp_vec.at(1);
    cartesian_displacement_goal.angular_disp.z = angular_disp_vec.at(2);

    // Load planning group from the parameter server
    if (!ros::param::get("/cartesian_displacement_test_client/planning_group", cartesian_displacement_goal.planning_group))
    {
        ROS_ERROR("Failed to load planning_group parameter.");
        return 1;
    }

    // std_msgs::Int32 n_wps;
    // if (!ros::param::get("/cartesian_displacement_test_client/number_of_waypoints", n_wps.data))
    // {
    //     ROS_ERROR("Failed to load nwps parameter.");
    //     return 1;
    // }
    // cartesian_displacement_goal.number_of_waypoints = n_wps.data;

    std::vector<double> initial_configuration;
    if (!ros::param::get("/cartesian_displacement_test_client/initial_configuration", cartesian_displacement_goal.initial_configuration))
    {
        ROS_ERROR("Failed to load initial_configuration parameter.");
        return 1;
    }

    // Send the goal to the action server
    ROS_INFO("Sending goal to SlerpDisplacementPlan action server...");
    cartesian_plan_displacement_client.sendGoal(cartesian_displacement_goal);

    // Wait for the action to complete (you can add a timeout here)
    bool finished_before_timeout = cartesian_plan_displacement_client.waitForResult();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = cartesian_plan_displacement_client.getState();
        ROS_INFO("SlerpPlan action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_ERROR("SlerpPlan action did not complete before the timeout.");
    }
    ros::waitForShutdown();

    return 0;
}