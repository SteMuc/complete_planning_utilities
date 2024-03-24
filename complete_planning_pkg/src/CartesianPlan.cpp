/**
 * @file CartesianPlan.cpp
 * @brief Implementation of Cartesian action server classes for Cartesian planning.
 *
 * This file contains the implementations of action server classes for Cartesian planning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/CartesianPlan.h>
#include <utils/constants.h>

namespace CartesianPlan
{

    void CartesianPlanActionServer::onGoal(GoalHandle gh)
    {
        ROS_INFO("Received new Cartesian Goal!");
        _cancelGoals[gh.getGoalID().id] = false;

        boost::shared_ptr<const complete_planning_msgs::CartesianPlanGoal> goal = gh.getGoal();
        std::string goal_id = gh.getGoalID().id;

        // Initialize Cartesian goal
        this->goal_pose = goal->goal_pose;
        this->initial_configuration = goal->initial_configuration;
        this->planning_group = goal->planning_group;

        if (this->isPoseFilled(this->goal_pose))
        {
            ROS_ERROR("The goal pose is empty. Did you fill correctly the goal pose?");
            gh.setRejected();
            _cancelGoals.erase(goal_id);
            return;
        }
        else
        {
            // Accepted the goal
            gh.setAccepted();
            ROS_INFO("Goal Accepted!");
            ROS_INFO("Processing goal: %s", goal_id.c_str());
            ROS_INFO("Goal Pose is: %f, %f, %f, %f, %f, %f, %f",
                     this->goal_pose.position.x,
                     this->goal_pose.position.y,
                     this->goal_pose.position.z,
                     this->goal_pose.orientation.x,
                     this->goal_pose.orientation.y,
                     this->goal_pose.orientation.z,
                     this->goal_pose.orientation.w);
        }

        // Initialize the MoveGroupInterface and joint model group
        moveit::planning_interface::MoveGroupInterface group(this->planning_group);
        const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(this->planning_group);
        auto joint_names = group.getVariableNames();

        // Set the goal pose for planning

        if (!this->isNullPose(this->goal_pose))
        {
            group.setPoseTarget(this->goal_pose);
        }
        else
        {
            ROS_ERROR("Empty goal configuration");
            gh.setAborted();
            _cancelGoals.erase(goal_id);
            return;
        }

        // Visual tools
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(group.getRobotModel()->getModelFrame());
        visual_tools.deleteAllMarkers();

        // Loading the remote control for visual tools and promting a message
        visual_tools.loadRemoteControl();
        visual_tools.trigger();

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
                ROS_ERROR("Wrong size: the number of joints in the goal is %zu, while it should be %zu", initial_configuration.size(), joint_names.size());
                gh.setAborted();
                _cancelGoals.erase(goal_id);
                return;
            }
        }
        else
        {
            // If initial_pose is null, set the start state to the current state
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

        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode planning_result = group.plan(plan);

        //
        complete_planning_msgs::CartesianPlanResult result;
        if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {   
            #ifdef VISUAL
            ROS_INFO("Visualizing the computed plan as trajectory line.");
            visual_tools.deleteAllMarkers();
            visual_tools.publishAxisLabeled(this->goal_pose, "goal pose");
            visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group->getLinkModel(group.getEndEffectorLink()), joint_model_group, rvt::YELLOW);
            visual_tools.trigger();
            #endif
            //
            result.planned_trajectory = plan.trajectory_;
            gh.setSucceeded(result);
            _cancelGoals.erase(goal_id);
        }
        else
        {
            ROS_INFO("Planning failed with error code: %d", planning_result.val);
            gh.setAborted(result);
            _cancelGoals.erase(goal_id);
            return;
        }
    };

    void CartesianPlanActionServer::onCancel(GoalHandle gh)
    {
        ROS_INFO("Cancel request has been received.");
        if (_cancelGoals.count(gh.getGoalID().id) > 0)
        {
            ROS_INFO("Found goal to cancel");
            _cancelGoals[gh.getGoalID().id] = true;
        }
    };

    bool CartesianPlanActionServer::isPoseFilled(const geometry_msgs::Pose &pose)
    {
        // Check if position and orientation arrays have 3 and 4 elements respectively
        return pose.position.x != 0.0 && pose.position.y != 0.0 && pose.position.z != 0.0 &&
               pose.orientation.x != 0.0 && pose.orientation.y != 0.0 && pose.orientation.z != 0.0 &&
               pose.orientation.w != 0.0;
    };
}
