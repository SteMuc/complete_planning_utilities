/**
 * @file JointPlan.h
 * @brief Header file for various ROS Joint action server.
 * @author Alessandro Palleschi, Stefano Angeli
 */

#ifndef JOINTSERVER_H
#define JOINTSERVER_H

#include <ros/ros.h>
#include <memory> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

// Joint plan Action headers
#include <complete_planning_msgs/JointPlanAction.h>
#include <complete_planning_msgs/JointPlanGoal.h>
#include <complete_planning_msgs/JointPlanResult.h>
#include <complete_planning_msgs/JointPlanFeedback.h>

#include <Eigen/Dense>

typedef actionlib::ServerGoalHandle<complete_planning_msgs::JointPlanAction> GoalHandle;

/**
 * @class JointPlanActionServer
 * @brief Base class for JointPlan Action Server.
 *
 */
class JointPlanActionServer
{
protected:
    ros::NodeHandle _nh;
    actionlib::ActionServer<complete_planning_msgs::JointPlanAction> _as;
    bool _preempted;
    std::map<std::string, bool> _cancelGoals;

    // Present joint configuration and computed trajectory
    std::vector<double> initial_configuration;
    std::vector<double> goal_configuration;
    std::string planning_group;

    trajectory_msgs::JointTrajectory computed_trajectory;

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
    JointPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : _nh(nh), _as(_nh, action_name, boost::bind(&JointPlanActionServer::onGoal, this, _1),
                       boost::bind(&JointPlanActionServer::onCancel, this, _1),
                       false)
    {
        _as.start();
        ROS_INFO("JointPlanActionServer has been started");
    };

    ~JointPlanActionServer(){
        // Nothin to do here
    };

    void onGoal(GoalHandle gh);
    void onCancel(GoalHandle gh);
};

#endif // JOINTSERVER_H