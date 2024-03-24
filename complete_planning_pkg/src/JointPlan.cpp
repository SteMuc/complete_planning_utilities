/**
 * @file JointPlan.cpp
 * @brief Implementation of Joint action server classes for Joint planning.
 *
 * This file contains the implementations of action server classes for Jointplanning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/JointPlan.h>
#include <utils/constants.h>

namespace JointPlan
{

    void JointPlanActionServer::onGoal(GoalHandle gh)
    {
        ROS_INFO("Received new Joint Goal!");
        _cancelGoals[gh.getGoalID().id] = false;

        boost::shared_ptr<const complete_planning_msgs::JointPlanGoal> goal = gh.getGoal();
        std::string goal_id = gh.getGoalID().id;

        // Initialize Joint Goal
        this->goal_configuration = goal->goal_configuration;
        this->initial_configuration = goal->initial_configuration;
        this->planning_group = goal->planning_group;

        // Initialize the MoveGroupInterface and joint model group
        moveit::planning_interface::MoveGroupInterface group(this->planning_group);
        const robot_state::JointModelGroup *joint_model_group =
            group.getCurrentState()->getJointModelGroup(this->planning_group);
        auto joint_names = group.getVariableNames();

        if (joint_names.size() == this->goal_configuration.size())
        {
            group.setJointValueTarget(this->goal_configuration);
            ROS_INFO("Goal Accepted!");
            gh.setAccepted();
        }
        else
        {
            ROS_ERROR("WRONG SIZE: the number of joints in the goal is %zu, while it should be %zu", this->goal_configuration.size(), joint_names.size());
            gh.setRejected();
            _cancelGoals.erase(goal_id);
            return;
        }

        // Create a start state from the initial configuration if provided
        if (!this->isNullPose(this->initial_configuration))
        {
            ROS_INFO("Setting Initial Position");
            if (joint_names.size() == this->initial_configuration.size())
            {
                moveit::core::RobotStatePtr current_state = group.getCurrentState();
                current_state->setJointGroupPositions(this->planning_group, this->initial_configuration);
                group.setStartState(*current_state);
            }
            else
            {
                ROS_ERROR("WRONG SIZE: the number of joints in the goal is %zu, while it should be %zu", this->initial_configuration.size(), joint_names.size());
                gh.setAborted();
                _cancelGoals.erase(goal_id);
                return;
            }
        }
        else
        {
            // If initial_pose is null, set the start state to the current state
            ROS_INFO("No intial position specified, using the current state");
            moveit::core::RobotStatePtr current_state = group.getCurrentState();
            group.setStartState(*current_state); // Use current_state here
        }

        if (_cancelGoals[goal_id])
        {
            ROS_INFO("Canceled the goal.");
            gh.setCanceled();
            _cancelGoals.erase(goal_id);
            return;
        }

        complete_planning_msgs::JointPlanResult result;

        // Visual tools
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(group.getRobotModel()->getModelFrame());
        visual_tools.deleteAllMarkers();

        // Loading the remote control for visual tools and promting a message
        visual_tools.loadRemoteControl();
        visual_tools.trigger();

        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode planning_result = group.plan(plan);

        if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            #ifdef VISUAL
            ROS_INFO("Visualizing the computed plan as trajectory line.");
            visual_tools.deleteAllMarkers();
            visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group->getLinkModel(group.getEndEffectorLink()), joint_model_group, rvt::YELLOW);
            visual_tools.trigger();
            #endif

            result.planned_trajectory = plan.trajectory_;
            gh.setSucceeded(result);
            ROS_INFO("Set Succeeded");
            _cancelGoals.erase(goal_id);
        }
        else
        {
            gh.setAborted(result);
            ROS_ERROR("Planning failed with error code: %d", planning_result.val);
            _cancelGoals.erase(goal_id);
            return;
        }
    };

    void JointPlanActionServer::onCancel(GoalHandle gh)
    {
        ROS_INFO("Cancel request has been received.");
        if (_cancelGoals.count(gh.getGoalID().id) > 0)
        {
            ROS_INFO("Found goal to cancel");
            _cancelGoals[gh.getGoalID().id] = true;
        }
    };
}
