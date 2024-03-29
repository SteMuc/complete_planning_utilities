/**
 * @file ExecutePlan.h
 * @brief Header file for ExecutePlan action servers.
 * @author Alessandro Palleschi, Stefano Angeli
 */

#ifndef ACTIONS_H
#define ACTIONS_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

// Execute plan Action headers
#include <complete_planning_msgs/ExecutePlanAction.h>
#include <complete_planning_msgs/ExecutePlanGoal.h>
#include <complete_planning_msgs/ExecutePlanResult.h>
#include <complete_planning_msgs/ExecutePlanFeedback.h>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

/**
 * @class ExecutePlanActionServer
 * @brief Class for executing a plan using an action server.
 */

namespace ExecutePlan
{
    typedef actionlib::ServerGoalHandle<complete_planning_msgs::ExecutePlanAction> GoalHandle;

    class ExecutePlanActionServer
    {
    public:
        /**
         * @brief Constructor for the Execute Plan Action Server.
         * @param nh The ROS NodeHandle.
         * @param action_name The name of the action server.
         */
        ExecutePlanActionServer(ros::NodeHandle nh, const std::string &action_name) : nh_(nh),
                                                                                      as_(nh_, action_name, boost::bind(&ExecutePlanActionServer::onGoal, this, _1), false)
        {
            as_.start();
        }

        /**
         * @brief Callback function for executing the Execute Plan action.
         * @param goal The goal message for the action.
         */
        void onGoal(GoalHandle gh);

        /**
         * @brief Callback function for handling the completion of the trajectory execution.
         * @param state The state of the goal execution.
         */

    private:
        ros::NodeHandle nh_;
         actionlib::ActionServer<complete_planning_msgs::ExecutePlanAction> as_;
    };
}
#endif