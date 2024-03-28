/**
 * @file ExecutePlan.cpp
 * @brief Implementation of Execute action server classes for Execute the planned trajectory.
 *
 * This file contains the implementations of action server classes for Execute the planned trajectory
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/ExecutePlan.h>
#include <utils/constants.h>

namespace ExecutePlan
{
    void ExecutePlanActionServer::onGoal(GoalHandle gh)
    {
        boost::shared_ptr<const complete_planning_msgs::ExecutePlanGoal> goal = gh.getGoal();
        
        // Accept the goal
        gh.setAccepted();
        std::string move_group_name = goal->move_group_name;
        ROS_INFO("Received goal for MoveGroup: %s", move_group_name.c_str());

        complete_planning_msgs::ExecutePlanResult result;

        // Extract the Plan from the custom action request
        const moveit_msgs::RobotTrajectory &robot_traj = goal->motion_plan;

        // Use MoveIt to execute the Plan
        moveit::planning_interface::MoveGroupInterface move_group(move_group_name);
        moveit::core::MoveItErrorCode exec_result = move_group.execute(robot_traj);

        if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            gh.setSucceeded(result);
        else
        {
            gh.setAborted(result);
            ROS_ERROR("Planning failed with error code: %d", exec_result.val);
            return;
        }
    }
}