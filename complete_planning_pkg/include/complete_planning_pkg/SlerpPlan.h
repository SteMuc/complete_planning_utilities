/**
 * @file SlerpPlan.h
 * @brief Header file for ROS Slerp action server. Uses SLERP interpolation between presnt ee pose and goal pose to create homogeneous motion.
 * @author Alessandro Palleschi, Stefano Angeli
 */

#ifndef SLERPSERVER_H
#define SLERPSERVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

// Cartesian plan Action headers
#include <complete_planning_msgs/SlerpPlanAction.h>
#include <complete_planning_msgs/SlerpPlanGoal.h>
#include <complete_planning_msgs/SlerpPlanResult.h>
#include <complete_planning_msgs/SlerpPlanFeedback.h>
#include <Eigen/Dense>

// Defines
#define DEBUG 1 // Prints out additional stuff

namespace SlerpPlan
{
    typedef actionlib::ServerGoalHandle<complete_planning_msgs::SlerpPlanAction> GoalHandle;

    /**
     * @class SlerpPlanActionServer
     * @brief Base class for Slerp Plan Action Server.
     *
     */

    class SlerpPlanActionServer
    {
    protected:
        ros::NodeHandle _nh;
        actionlib::ActionServer<complete_planning_msgs::SlerpPlanAction> _as;
        std::map<std::string, bool> _cancelGoals;
        //
        geometry_msgs::Pose initial_pose;
        geometry_msgs::Pose goal_pose;
        std::string planning_group;
        std::vector<double> initial_configuration;
        std_msgs::Int32 n_wp;
        //
        tf::TransformListener tf_listener;
        tf::StampedTransform stamp_ee_transform;
        Eigen::Affine3d end_effector_state;
        Eigen::Affine3d startAff;
        Eigen::Affine3d goalAff;

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
        SlerpPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
            : _nh(nh), _as(_nh, action_name, boost::bind(&SlerpPlanActionServer::onGoal, this, _1),
                           boost::bind(&SlerpPlanActionServer::onCancel, this, _1),
                           false)
        {
            _as.start();
            ROS_INFO("SlerpPlanActionServer has been started");
        };

        ~SlerpPlanActionServer(){
            // Nothin to do here
        };

        void onGoal(GoalHandle gh);
        void onCancel(GoalHandle gh);
        bool isPoseFilled(const geometry_msgs::Pose &pose);
        bool isReallyNullPose(const geometry_msgs::Pose &pose);
        void computeWaypointsFromPoses(const Eigen::Affine3d &start_pose, const Eigen::Affine3d &goal_pose, std::vector<geometry_msgs::Pose> &waypoints);
    };
}
#endif