#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/client_helpers.h>
#include <map>
#include <string>

// Cartesian plan Action headers
#include <complete_planning_msgs/CartesianPlanAction.h>
#include <complete_planning_msgs/CartesianPlanGoal.h>

// Joint plan Action headers
#include <complete_planning_msgs/JointPlanAction.h>
#include <complete_planning_msgs/JointPlanGoal.h>


// Slerp plan Action headers
#include <complete_planning_msgs/SlerpPlanAction.h>
#include <complete_planning_msgs/SlerpPlanGoal.h>

// Grasp action headers
#include <complete_planning_msgs/GraspAction.h>
#include <complete_planning_msgs/GraspGoal.h>

// Release Action headers
#include <complete_planning_msgs/ReleaseAction.h>
#include <complete_planning_msgs/ReleaseGoal.h>

// Execute Plan Action headers
#include <complete_planning_msgs/ExecutePlanAction.h>
#include <complete_planning_msgs/ExecutePlanGoal.h>

#include <geometry_msgs/Pose.h>
#include <string>

typedef actionlib::ClientGoalHandle<complete_planning_msgs::CartesianPlanAction> GoalHandle;

/**
 * @class CartesianPlanActionClient
 * @brief Base class for Cartesian Plan Action Client.
 *
 */
class CartesianPlanActionClient
{
protected:
    ros::NodeHandle _nh;
    actionlib::ActionClient<complete_planning_msgs::CartesianPlanAction> _ac;

public:
    std::map<int, GoalHandle *> _goalHandles;

    CartesianPlanActionClient(ros::NodeHandle &nh, const std::string &action_name)
        : _nh(nh), _ac(action_name)
    {
        _ac.waitForActionServerToStart();
        ROS_INFO("Cartesian Plan Action Server is up, we can send goals");
    }
    ~CartesianPlanActionClient()
    {
        // Nothing to do here
    }

    GoalHandle sendGoal(geometry_msgs::Pose goal_pose, std::vector<double> initial_configuration, std::string planning_group);
    void onFeedback(const GoalHandle gh, const complete_planning_msgs::CartesianPlanFeedbackConstPtr &feedback);
    void onTransition(const GoalHandle gh);
};