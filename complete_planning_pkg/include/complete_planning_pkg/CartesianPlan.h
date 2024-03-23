/**
 * @file CartesianPlan.h
 * @brief Header file for ROS Cartesian action server.
 * @author Alessandro Palleschi, Stefano Angeli
 */

#ifndef CARTESIANSERVER_H
#define CARTESIANSERVER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

// Cartesian plan Action headers
#include <complete_planning_msgs/CartesianPlanAction.h>
#include <complete_planning_msgs/CartesianPlanGoal.h>
#include <complete_planning_msgs/CartesianPlanResult.h>
#include <complete_planning_msgs/CartesianPlanFeedback.h>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace CartesianPlan
{

    typedef actionlib::ServerGoalHandle<complete_planning_msgs::CartesianPlanAction> GoalHandle;

    /**
     * @class CartesianPlanActionServer
     * @brief Base class for Cartesian Plan Action Server.
     *
     */

    class CartesianPlanActionServer
    {
    protected:
        ros::NodeHandle _nh;
        actionlib::ActionServer<complete_planning_msgs::CartesianPlanAction> _as;
        bool _preempted;
        std::map<std::string, bool> _cancelGoals;

        std::vector<double> initial_configuration;
        geometry_msgs::Pose goal_pose;
        std::string planning_group;

        /**
         * @brief Check if a given pose is a null pose.
         * @param pose The pose to check.
         * @return True if the pose is null, false otherwise.
         */
        inline bool isNullPose(const geometry_msgs::Pose &pose)
        {
            return (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
                    pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0);
        };

        /**
         * @brief Check if a given joint trajectory point is null.
         * @param joint_configuration The joint configuration to check.
         * @return True if the joint configuration is null, false otherwise.
         */
        inline bool isNullPose(const std::vector<double> &joint_configuration)
        {
            return joint_configuration.empty();
        }

    public:
        CartesianPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
            : _nh(nh), _as(_nh, action_name, boost::bind(&CartesianPlanActionServer::onGoal, this, _1),
                           boost::bind(&CartesianPlanActionServer::onCancel, this, _1),
                           false)
        {
            _as.start();
            ROS_INFO("CartesianPlanActionServer has been started");
        };

        ~CartesianPlanActionServer()
        {
            // Nothin to do here
        }

        void onGoal(GoalHandle gh);
        void onCancel(GoalHandle gh);
        bool isPoseFilled(const geometry_msgs::Pose &pose);
    };
}
#endif // CARTESIANSERVER_H