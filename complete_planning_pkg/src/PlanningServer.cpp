/**
 * @file PlanningServer.cpp
 * @brief Implementation of a ROS node that start action servers for Cartesian planning.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <ros/ros.h>
#include "complete_planning_pkg/CartesianPlan.h"
#include "complete_planning_pkg/JointPlan.h"
#include "complete_planning_pkg/SlerpPlan.h"
#include "complete_planning_pkg/SlerpPlanDisplacement.h"
#include "complete_planning_pkg/CartesianPlanDisplacement.h"
#include "complete_planning_pkg/ExecutePlan.h"


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_server_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Create instances of your action server classes
    CartesianPlan::CartesianPlanActionServer cartesian_server(nh, "cartesian_plan_action");
    JointPlan::JointPlanActionServer joint_server(nh, "joint_plan_action");
    SlerpPlan::SlerpPlanActionServer slerp_server(nh, "slerp_plan_action");
    SlerpPlanDisplacement::SlerpPlanDisplacementActionServer slerp_displacement_server(nh, "slerp_plan_displacement_action");
    CartesianPlanDisplacement::CartesianPlanDisplacementActionServer cartesian_displacement_server(nh, "cartesian_plan_displacement_action");
    ExecutePlan::ExecutePlanActionServer execute_plan_server(nh,"execute_plan_action" );
    ros::waitForShutdown();
    return 0;
}