#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "complete_planning_pkg/task_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_planner_node");
    ros::NodeHandle nh;

    TaskHandler task_handler(nh);

    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list))
    {
        if (task_list.size() > 0)
        {
            for (int i = 0; i < task_list.size(); ++i)
            {
                XmlRpc::XmlRpcValue task = task_list[i];
                PlanningGoal planning_goal = task_handler.convertYamlToGoal(task);
                if (!task_handler.plan(planning_goal))
                    break;
            }
        }
        else
        {
            ROS_ERROR("Task List is Empty!");
            return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to retrieve the /task_list parameter from the parameter server.");
        return 1;
    }
    while (!task_handler.isExecutionQueueEmpty() && task_handler.isExecutionThreadRunning())
    {
        ros::spinOnce();            // Allow ROS to process callbacks
        ros::Duration(0.1).sleep(); // Sleep for a while
    }
    task_handler.stopExecutionThread();
    nh.shutdown();
    return 0;
}
