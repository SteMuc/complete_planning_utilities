/**
 * @file CartesianPlanDisplacement.cpp
 * @brief Implementation of Cartesian Plan Displacement action server classes for Cartesian planning.
 *
 * This file contains the implementations of action server classes for Cartesian Displacement planning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/CartesianPlanDisplacement.h>
#include <utils/constants.h>

namespace CartesianPlanDisplacement
{
    void CartesianPlanDisplacementActionServer::onGoal(GoalHandle gh)
    {
        ROS_INFO("Received new Slerp Goal!");
        _cancelGoals[gh.getGoalID().id] = false;

        boost::shared_ptr<const complete_planning_msgs::CartesianPlanDisplacementGoal> goal = gh.getGoal();
        std::string goal_id = gh.getGoalID().id;

        // Initialize Slerp Displacement goal
        this->pos_disp = goal->pos_disp;
        this->angular_disp = goal->angular_disp;
        this->planning_group = goal->planning_group;
        this->initial_configuration = goal->initial_configuration;

        if (this->isDisplacementFilled(this->pos_disp, this->angular_disp))
        {
            ROS_ERROR("The displacement is empty. Did you fill correctly the displacement?");
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
            ROS_INFO("The Position Displacement is: %f, %f, %f. The Angular Displacement is: %f, %f, %f",
                     this->pos_disp.x,
                     this->pos_disp.y,
                     this->pos_disp.z,
                     this->angular_disp.x,
                     this->angular_disp.y,
                     this->angular_disp.z);
        }

        // Initialize the MoveGroupInterface
        moveit::planning_interface::MoveGroupInterface group(this->planning_group);

        // Getting the robot joint model
        const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(this->planning_group);
        // robot_state::RobotState start_state(*group.getCurrentState());
        auto joint_names = group.getVariableNames();

        // Convert the Displacement into Eigen Affine3d
        this->DisplacementAff = this->convert_to_affine(this->pos_disp, this->angular_disp);

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
                this->end_effector_state = current_state->getGlobalLinkTransform(group.getEndEffectorLink());
                this->startAff = this->end_effector_state;
                
                if (DEBUG)
                {
                    ROS_WARN("Setting initial Position externally");
                    ROS_INFO_STREAM("The Start Initial Translation of the robot is:" << this->startAff.translation() << "\n");
                    Eigen::Quaterniond initial_quat(this->startAff.rotation());
                    ROS_INFO_STREAM("The Start Initial Quaternion of the robot is:" << initial_quat.coeffs() << "\n");
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
            ROS_WARN("Planning from the current state");
            moveit::core::RobotStatePtr current_state = group.getCurrentState();
            group.setStartStateToCurrentState();
            this->end_effector_state = current_state->getGlobalLinkTransform(group.getEndEffectorLink());
            this->startAff = this->end_effector_state;

            if (DEBUG)
            {
                ROS_WARN("Setting initial Position internally");
                ROS_INFO_STREAM("The Start Initial Translation of the robot is:" << this->startAff.translation() << "\n");
                Eigen::Quaterniond initial_quat(this->startAff.rotation());
                ROS_INFO_STREAM("The Start Initial Quaternion of the robot is:" << initial_quat.coeffs() << "\n");
            }
        }

        // If the goal is relative, get the global goal and start poses by multiplying it with ee pose (end_effector_state)
        // this->startAff = this->startAff * this->DisplacementAff;
        Eigen::Quaterniond quat_start_aff(this->startAff.linear());
        std::cout << "Start Aff Translation: " << this->startAff.translation() << std::endl;
        std::cout << "Start Quaternion is:" << quat_start_aff.x() << " " << quat_start_aff.y() << " " << quat_start_aff.z() << " "
                  << " " << quat_start_aff.z() << " " << std::endl;

        this->goalAff = this->startAff * this->DisplacementAff;

        // Set the Pose Target
        tf::poseEigenToMsg(this->goalAff, this->goal_pose);
        group.setPoseTarget(this->goal_pose);

        Eigen::Quaterniond quat_goal_aff(this->goalAff.linear());
        std::cout << "Goal Aff Translation: " << this->goalAff.translation() << std::endl;
        std::cout << "Goal Quaternion is:" << quat_goal_aff.x() << " " << quat_goal_aff.y() << " " << quat_goal_aff.z() << " "
                  << " " << quat_goal_aff.z() << " " << std::endl;

        ROS_INFO("Goal Pose is: %f, %f, %f, %f, %f, %f, %f",
                 this->goal_pose.position.x,
                 this->goal_pose.position.y,
                 this->goal_pose.position.z,
                 this->goal_pose.orientation.x,
                 this->goal_pose.orientation.y,
                 this->goal_pose.orientation.z,
                 this->goal_pose.orientation.w);

        // Visual tools
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(group.getRobotModel()->getModelFrame());
        visual_tools.deleteAllMarkers();

        // Loading the remote control for visual tools and promting a message
        visual_tools.loadRemoteControl();
        visual_tools.trigger();

        //
        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode planning_result = group.plan(plan);

        //
        complete_planning_msgs::CartesianPlanDisplacementResult result;
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

    void CartesianPlanDisplacementActionServer::onCancel(GoalHandle gh)
    {
        ROS_INFO("Cancel request has been received.");
        if (_cancelGoals.count(gh.getGoalID().id) > 0)
        {
            ROS_INFO("Found goal to cancel");
            _cancelGoals[gh.getGoalID().id] = true;
        }
    };

    bool CartesianPlanDisplacementActionServer::isPoseFilled(const geometry_msgs::Pose &pose)
    {
        // Check if position and orientation arrays have 3 and 4 elements respectively
        return pose.position.x != 0.0 && pose.position.y != 0.0 && pose.position.z != 0.0 &&
               pose.orientation.x != 0.0 && pose.orientation.y != 0.0 && pose.orientation.z != 0.0 &&
               pose.orientation.w != 0.0;
    };

    bool CartesianPlanDisplacementActionServer::isReallyNullPose(const geometry_msgs::Pose &pose)
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

    Eigen::Affine3d CartesianPlanDisplacementActionServer::convert_to_affine(geometry_msgs::Point &pos_disp, geometry_msgs::Point &angular_disp)
    {
        // Create Affine3d transformation
        Eigen::Affine3d affineTransformation = Eigen::Affine3d::Identity();

        // Orientation
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(angular_disp.x * M_PI, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(angular_disp.y * M_PI, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(angular_disp.z * M_PI, Eigen::Vector3d::UnitZ());

        // Translation
        Eigen::Vector3d translation(pos_disp.x, pos_disp.y, pos_disp.z);

        // Set the rotation and translation
        affineTransformation.linear() = rotation_matrix;
        affineTransformation.translation() = translation;

        // Print the Affine3d transformation
        std::cout << "Affine transformation:\n"
                  << affineTransformation.matrix() << std::endl;

        return affineTransformation;
    };
}
