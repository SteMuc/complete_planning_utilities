/**
 * @file SlerpPlan.cpp
 * @brief Implementation of Slerp action server classes for Slerp planning.
 *
 * This file contains the implementations of action server classes for Slerp planning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/SlerpPlan.h>
#include <utils/constants.h>

namespace SlerpPlan
{
    void SlerpPlanActionServer::onGoal(GoalHandle gh)
    {
        ROS_INFO("Received new Slerp Goal!");
        _cancelGoals[gh.getGoalID().id] = false;

        boost::shared_ptr<const complete_planning_msgs::SlerpPlanGoal> goal = gh.getGoal();
        std::string goal_id = gh.getGoalID().id;

        // Initialize Cartesian goal
        this->goal_pose = goal->goal_pose;
        this->planning_group = goal->planning_group;
        this->initial_configuration = goal->initial_configuration;
        this->n_wp.data = goal->number_of_waypoints;

        std::cout << "n_wp.data" << this->n_wp.data << std::endl;
        if (this->isPoseFilled(this->goal_pose))
        {
            ROS_ERROR("The goal pose is empty. Did you fill correctly the goal pose?");
            _cancelGoals.erase(goal_id);
            gh.setRejected();
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

        // Initialize the MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface group(this->planning_group);

        // Getting the robot joint model
        const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(this->planning_group);
        robot_state::RobotState start_state(*group.getCurrentState());
        auto joint_names = group.getVariableNames();

        // Printing the planning group frame and the group ee frame
        if (DEBUG)
        {
            ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
            ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());
        }

        if (!this->isNullPose(this->initial_configuration))
        {
            ROS_INFO("Setting Initial Position");
            if (joint_names.size() == this->initial_configuration.size())
            {
                moveit::core::RobotStatePtr current_state = group.getCurrentState();
                current_state->setJointGroupPositions(this->planning_group, this->initial_configuration);
                group.setStartState(*current_state);
                this->startAff = current_state->getGlobalLinkTransform(group.getEndEffectorLink());
                if (DEBUG)
                {
                    ROS_INFO_STREAM("The Start Initial Translation of the robot is:" << this->startAff.translation() << "\n");
                    Eigen::Quaterniond initial_quat(this->startAff.rotation());
                    ROS_INFO_STREAM("The Start Initial Quaternion of the robot is:" << initial_quat.coeffs() << "\n");
                    ROS_WARN("Setting initial Position externally");
                }
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
            group.setStartStateToCurrentState();
            this->startAff = current_state->getGlobalLinkTransform(group.getEndEffectorLink());
            if (DEBUG)
            {
                ROS_INFO_STREAM("The Start Initial Translation of the robot is:" << this->startAff.translation() << "\n");
                Eigen::Quaterniond initial_quat(this->startAff.rotation());
                ROS_INFO_STREAM("The Start Initial Quaternion of the robot is:" << initial_quat.coeffs() << "\n");
                ROS_WARN("Setting initial Position internally");
            }
        }

        // Setting the goal pose
        tf::poseMsgToEigen(goal_pose, this->goalAff);

        // Visual tools
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(group.getRobotModel()->getModelFrame());
        visual_tools.deleteAllMarkers();

        // Loading the remote control for visual tools and promting a message
        visual_tools.loadRemoteControl();
        visual_tools.trigger();

        // Calling the waypoint creator with start and goal poses
        std::vector<geometry_msgs::Pose> cart_waypoints;
        this->computeWaypointsFromPoses(this->startAff, this->goalAff, cart_waypoints);

        // Scale the velocity and acceleration of the computed trajectory
        const double velocity_scaling_factor = 0.5;     // Set your desired velocity scaling factor
        const double acceleration_scaling_factor = 0.2; // Set your desired acceleration scaling factor

        // Planning for the waypoints path
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = group.computeCartesianPath(cart_waypoints, 0.01, 0.0, trajectory);

        robot_trajectory::RobotTrajectory rt(start_state.getRobotModel(), this->planning_group);

        rt.setRobotTrajectoryMsg(start_state, trajectory);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;

        bool success = totg.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);
        ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");
        rt.getRobotTrajectoryMsg(trajectory);

#ifdef VISUAL
        ROS_INFO("Visualizing the computed plan as trajectory line.");
        visual_tools.publishAxisLabeled(cart_waypoints.back(), "goal pose");
        visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel(group.getEndEffectorLink()), joint_model_group, rvt::YELLOW);
        visual_tools.trigger();
#endif

        complete_planning_msgs::SlerpPlanResult result;
        // If complete path is not achieved return false, true otherwise
        if (fraction != 1.0)
        {
            ROS_INFO("Plan (cartesian path) (%.2f%% acheived) is:", fraction * 100.0);
            ROS_INFO("Fraction is less than one!");
            ROS_INFO("Set Aborted!");
            _cancelGoals.erase(goal_id);
            gh.setAborted(result);
            return;
        }
        else
        {
            ROS_INFO("Set succeeded!!");
            result.planned_trajectory = trajectory;
#ifdef PROMPT
            namespace rvt = rviz_visual_tools;
            moveit_visual_tools::MoveItVisualTools visual_tools(group.getRobotModel()->getModelFrame());
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue and execute the Slerp Plan");
#endif
            _cancelGoals.erase(goal_id);
            gh.setSucceeded(result);
        }
    };

    void SlerpPlanActionServer::onCancel(GoalHandle gh)
    {
        ROS_INFO("Cancel request has been received.");
        if (_cancelGoals.count(gh.getGoalID().id) > 0)
        {
            ROS_INFO("Found goal to cancel");
            _cancelGoals[gh.getGoalID().id] = true;
        }
    };

    bool SlerpPlanActionServer::isPoseFilled(const geometry_msgs::Pose &pose)
    {
        // Check if position and orientation arrays have 3 and 4 elements respectively
        return pose.position.x != 0.0 && pose.position.y != 0.0 && pose.position.z != 0.0 &&
               pose.orientation.x != 0.0 && pose.orientation.y != 0.0 && pose.orientation.z != 0.0 &&
               pose.orientation.w != 0.0;
    };

    bool SlerpPlanActionServer::isReallyNullPose(const geometry_msgs::Pose &pose)
    {
        geometry_msgs::Point pos = pose.position;
        geometry_msgs::Quaternion quat = pose.orientation;

        if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0 &&
            quat.x == 0.0 && quat.y == 0.0 && quat.z == 0.0 && quat.w == 1.0)
        {
            return true;
        }

        return false;
    };

    // Computes waypoints using SLERP from two poses
    void SlerpPlanActionServer::computeWaypointsFromPoses(const Eigen::Affine3d &start_pose, const Eigen::Affine3d &goal_pose, std::vector<geometry_msgs::Pose> &waypoints)
    {

        // Compute waypoints as linear interpolation (SLERP for rotations) between the two poses
        Eigen::Affine3d wp_eigen;
        geometry_msgs::Pose current_wp;

        Eigen::Vector3d start_vec = start_pose.translation();
        Eigen::Vector3d goal_vec = goal_pose.translation();
        Eigen::Vector3d diff_vec = goal_vec - start_vec;

        Eigen::Quaterniond start_quat(start_pose.linear());
        Eigen::Quaterniond goal_quat(goal_pose.linear());
        Eigen::Quaterniond diff_quat;

        // Setting the number of wp according to diff_vec
        int real_n_wp = std::floor(diff_vec.norm() * this->n_wp.data);
        ROS_INFO_STREAM(" Real Num Waypoints: " << real_n_wp);
        if (DEBUG)
        {
            ROS_INFO_STREAM("The norm of the diff_vec is " << diff_vec.norm() << ", so the new number of waypoints is " << real_n_wp << ".");
        }
        for (int j = 1; j <= real_n_wp; j++)
        {
            wp_eigen.translation() = start_vec + (diff_vec / real_n_wp) * j;
            diff_quat = start_quat.slerp(double(j) / double(real_n_wp), goal_quat);
            wp_eigen.linear() = diff_quat.toRotationMatrix();
            tf::poseEigenToMsg(wp_eigen, current_wp);
            waypoints.push_back(current_wp);

            if (DEBUG)
            {
                std::cout << "WAYPOINT NUMBER " << j << "." << std::endl;
                std::cout << "WP R: \n"
                          << wp_eigen.linear() << std::endl;
                std::cout << "WP t: \n"
                          << wp_eigen.translation() << std::endl;
            }
        }
    };
}
