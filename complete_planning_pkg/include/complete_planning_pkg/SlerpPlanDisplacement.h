/**
 * @file SlerpPlanDisplacement.h
 * @brief Header file for ROS Slerp Displacement action server. Uses SLERP interpolation between presnt ee pose and goal pose to create homogeneous motion.
 * @author Alessandro Palleschi, Stefano Angeli
 */

#ifndef SLERPDISPLACEMENTSERVER_H
#define SLERPDISPLACEMENTSERVER_H

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
#include <complete_planning_msgs/SlerpPlanDisplacementAction.h>
#include <complete_planning_msgs/SlerpPlanDisplacementGoal.h>
#include <complete_planning_msgs/SlerpPlanDisplacementResult.h>
#include <complete_planning_msgs/SlerpPlanDisplacementFeedback.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Defines
#define DEBUG 1 // Prints out additional stuff

namespace SlerpPlanDisplacement
{
    typedef actionlib::ServerGoalHandle<complete_planning_msgs::SlerpPlanDisplacementAction> GoalHandle;

    /**
     * @class SlerpPlanActionServer
     * @brief Base class for Slerp Plan Action Server.
     *
     */

    class SlerpPlanDisplacementActionServer
    {
    protected:
        ros::NodeHandle _nh;
        actionlib::ActionServer<complete_planning_msgs::SlerpPlanDisplacementAction> _as;
        std::map<std::string, bool> _cancelGoals;
        //
        geometry_msgs::Point pos_disp;
        geometry_msgs::Point angular_disp;
        std::vector<double> initial_configuration;
        std::string planning_group;
        std_msgs::Int32 n_wp;
        //
        Eigen::Affine3d startAff;
        Eigen::Affine3d goalAff;
        Eigen::Affine3d DisplacementAff;

        /**
         * @brief Check if a given pose is a null pose.
         * @param pose The pose to check.
         * @return True if the pose is null, false otherwise.
         */
        inline bool isDisplacementFilled(const geometry_msgs::Point &pos_disp,const geometry_msgs::Point &angular_disp)
        {
            return (pos_disp.x == 0.0 && pos_disp.y == 0.0 && pos_disp.z == 0.0 &&
                    angular_disp.x == 0.0 && angular_disp.y == 0.0 && angular_disp.z == 0.0);
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
        SlerpPlanDisplacementActionServer(ros::NodeHandle &nh, const std::string &action_name)
            : _nh(nh), _as(_nh, action_name, boost::bind(&SlerpPlanDisplacementActionServer::onGoal, this, _1),
                           boost::bind(&SlerpPlanDisplacementActionServer::onCancel, this, _1),
                           false)
        {
            _as.start();
            ROS_INFO("SlerpPlanDisplacementActionServer has been started");
        };

        ~SlerpPlanDisplacementActionServer(){
            // Nothin to do here
        };
        Eigen::Affine3d convert_to_affine(geometry_msgs::Point &pos_disp, geometry_msgs::Point &angular_disp);
        void onGoal(GoalHandle gh);
        void onCancel(GoalHandle gh);
        bool isPoseFilled(const geometry_msgs::Pose &pose);
        bool isReallyNullPose(const geometry_msgs::Pose &pose);
        void computeWaypointsFromPoses(const Eigen::Affine3d &start_pose, const Eigen::Affine3d &goal_pose, std::vector<geometry_msgs::Pose> &waypoints);
    };
}
#endif