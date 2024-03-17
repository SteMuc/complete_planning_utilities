/**
 * @file PlanningServer.cpp
 * @brief Implementation of a ROS node that start action servers for Cartesian planning.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */


#include <ros/ros.h>
#include "complete_planning_pkg/ActionServer.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_server_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    // Create instances of your action server classes
    CartesianPlanActionServer cartesian_server(nh, "cartesian_plan_action");

    // Spin to keep the node alive
    // ros::spin();
    ros::waitForShutdown();
    return 0;
}