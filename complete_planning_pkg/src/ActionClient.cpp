/**
 * @file ActionClient.cpp
 * @brief Implementation of action client classes for Cartesian planning.
 *
 * This file contains the implementations of action client classes for Cartesian planning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories by sending goals.
 *
 *
 * @author Alessandro Palleschi, Stefano Angeli
 * @email alessandropalleschi94@gmail.com, stefano.angeli@ing.unipi.it
 */

#include <complete_planning_pkg/ActionClient.h>

GoalHandle CartesianPlanActionClient::sendGoal(geometry_msgs::Pose goal_pose, std::vector<double> initial_configuration, std::string planning_group)
{
    complete_planning_msgs::CartesianPlanGoal goal;
    goal.goal_pose = goal_pose;
    goal.initial_configuration = initial_configuration;
    goal.planning_group = planning_group;

    GoalHandle goal_handle = _ac.sendGoal(goal,
                                          boost::bind(&CartesianPlanActionClient::onTransition, this, _1),
                                          boost::bind(&CartesianPlanActionClient::onFeedback, this, _1, _2));
    ROS_INFO("A goal has been sent");
    return goal_handle;
};

void CartesianPlanActionClient::onFeedback(const GoalHandle gh, const complete_planning_msgs::CartesianPlanFeedbackConstPtr &feedback){
    // TO DO
};

void CartesianPlanActionClient::onTransition(const GoalHandle gh)
{
    int index = 0;
    std::map<int, GoalHandle *>::iterator it = _goalHandles.begin();
    while (it != _goalHandles.end())
    {
        if (*(it->second) == gh)
        {
            index = it->first;
            break;
        }
        it++;
    }
    ROS_INFO("%d --- Transition callback", index);
    actionlib::CommState commState = gh.getCommState();

    ROS_INFO("%d: Comm state: %s", index, commState.toString().c_str());

    if (commState.toString() == "ACTIVE")
    {
        ROS_INFO("%d: Goal just went active", index);
    }
    else if (commState.toString() == "DONE")
    {
        ROS_INFO("%d: Goal is DONE", index);
        actionlib::TerminalState state = gh.getTerminalState();
        ROS_INFO("Client terminal state: %s", state.toString().c_str());
        complete_planning_msgs::CartesianPlanResultConstPtr result = gh.getResult();
    }
};
