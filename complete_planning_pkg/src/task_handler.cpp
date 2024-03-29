#include "complete_planning_pkg/task_handler.h"

TaskHandler::TaskHandler(ros::NodeHandle &nh)
    : nh_(nh),
      is_executing_(false),
      last_planned_configuration_(nullptr),
      cartesian_client_(new actionlib::SimpleActionClient<complete_planning_msgs::CartesianPlanAction>("cartesian_plan_action", true)),
      joint_client_(new actionlib::SimpleActionClient<complete_planning_msgs::JointPlanAction>("joint_plan_action", true)),
      slerp_client_(new actionlib::SimpleActionClient<complete_planning_msgs::SlerpPlanAction>("slerp_plan_action", true)),
      execute_client_(new actionlib::SimpleActionClient<complete_planning_msgs::ExecutePlanAction>("execute_plan_action", true))
{
    if (!cartesian_client_->waitForServer(ros::Duration(5.0)) ||
        !joint_client_->waitForServer(ros::Duration(5.0)) ||
        !execute_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("One or more action servers did not start within the timeout.");
        ros::shutdown();
    }
    execution_thread_ = std::thread(&TaskHandler::handleExecution, this);
    // Check if the thread is running and set run_thread accordingly
    run_thread = execution_thread_.joinable();
}

TaskHandler::~TaskHandler()
{
    stopExecutionThread();
}

geometry_msgs::Pose TaskHandler::parsePose(const XmlRpc::XmlRpcValue &pose_param)
{
    geometry_msgs::Pose pose;
    pose.position.x = static_cast<double>(pose_param["position"]["x"]);
    pose.position.y = static_cast<double>(pose_param["position"]["y"]);
    pose.position.z = static_cast<double>(pose_param["position"]["z"]);
    pose.orientation.x = static_cast<double>(pose_param["orientation"]["x"]);
    pose.orientation.y = static_cast<double>(pose_param["orientation"]["y"]);
    pose.orientation.z = static_cast<double>(pose_param["orientation"]["z"]);
    pose.orientation.w = static_cast<double>(pose_param["orientation"]["w"]);
    return pose;
}

PlanningGoal TaskHandler::convertYamlToGoal(const XmlRpc::XmlRpcValue &task_param)
{
    if (task_param.hasMember("type"))
    {
        std::string task_type = static_cast<std::string>(task_param["type"]);

        if (task_type == "CartesianPlan")
        {
            complete_planning_msgs::CartesianPlanGoal cartesian_goal;

            if (task_param.hasMember("goal"))
            {
                cartesian_goal.goal_pose = parsePose(task_param["goal"]);
            }
            else
            {
                ROS_ERROR("No goal specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("group"))
            {
                cartesian_goal.planning_group = static_cast<std::string>(task_param["group"]);
            }
            else
            {
                ROS_ERROR("No group specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("merge") && task_param["merge"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            {
                if (static_cast<bool>(task_param["merge"]))
                {
                    if (last_planned_configuration_)
                    {
                        std::cout << (*last_planned_configuration_).first << std::endl;
                        std::cout << cartesian_goal.planning_group << std::endl;

                        if ((*last_planned_configuration_).first == cartesian_goal.planning_group)
                        {
                            cartesian_goal.initial_configuration = (*last_planned_configuration_).second; // Use the shared vector
                        }
                        else
                        {
                            waitForExecutions();
                        }
                    }
                    else
                    {
                        ROS_ERROR("No previous configuration available for merge.");
                        return std::monostate{}; // Return an empty variant in case of an error
                    }
                }
                else
                {
                    waitForExecutions();
                }
            }

            return cartesian_goal;
        }
        else if (task_type == "JointPlan")
        {
            complete_planning_msgs::JointPlanGoal joint_goal;

            if (task_param.hasMember("goal"))
            {
                XmlRpc::XmlRpcValue goal = task_param["goal"];
                for (int i = 0; i < goal.size(); ++i)
                {
                    if (goal[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        double value = static_cast<double>(goal[i]);
                        joint_goal.goal_configuration.push_back(value);
                    }
                }
            }
            else
            {
                ROS_ERROR("No goal specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("group"))
            {
                joint_goal.planning_group = static_cast<std::string>(task_param["group"]);
            }
            else
            {
                ROS_ERROR("No group specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("merge") && task_param["merge"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            {
                if (static_cast<bool>(task_param["merge"]))
                {
                    if (last_planned_configuration_)
                    {
                        if ((*last_planned_configuration_).first == joint_goal.planning_group)
                        {
                            joint_goal.initial_configuration = (*last_planned_configuration_).second; // Use the shared vector
                        }
                        else
                        {
                            waitForExecutions();
                        }
                    }
                    else
                    {
                        ROS_ERROR("No previous configuration available for merge.");
                        return std::monostate{}; // Return an empty variant in case of an error
                    }
                }
                else
                {
                    waitForExecutions();
                }
            }

            return joint_goal;
        }
        else if (task_type == "SlerpPlan")
        {
            complete_planning_msgs::SlerpPlanGoal slerp_goal;

            if (task_param.hasMember("goal"))
            {
                slerp_goal.goal_pose = parsePose(task_param["goal"]);
            }
            else
            {
                ROS_ERROR("No goal specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("group"))
            {
                slerp_goal.planning_group = static_cast<std::string>(task_param["group"]);
            }
            else
            {
                ROS_ERROR("No group specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("number_of_waypoints"))
            {
                slerp_goal.number_of_waypoints = static_cast<int>(task_param["number_of_waypoints"]);
            }
            else
            {
                ROS_ERROR("No number of waypoints specified!");
                return std::monostate{}; // Return an empty variant in case of an error
            }

            if (task_param.hasMember("merge") && task_param["merge"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            {
                if (static_cast<bool>(task_param["merge"]))
                {
                    if (last_planned_configuration_)
                    {
                        std::cout << (*last_planned_configuration_).first << std::endl;
                        std::cout << slerp_goal.planning_group << std::endl;

                        if ((*last_planned_configuration_).first == slerp_goal.planning_group)
                        {
                            slerp_goal.initial_configuration = (*last_planned_configuration_).second; // Use the shared vector
                        }
                        else
                        {
                            waitForExecutions();
                        }
                    }
                    else
                    {
                        ROS_ERROR("No previous configuration available for merge.");
                        return std::monostate{}; // Return an empty variant in case of an error
                    }
                }
                else
                {
                    waitForExecutions();
                }
            }
            return slerp_goal;
        }
    }

    ROS_ERROR("Wrong task parameters!");
    return std::monostate{}; // Return an empty variant in case of an error
}

void TaskHandler::waitForExecutions()
{
    while (!isExecutionQueueEmpty() && isExecutionThreadRunning())
    {
        ros::spinOnce();             // Allow ROS to process callbacks
        ros::Duration(0.01).sleep(); // Sleep for a while
    }
}

bool TaskHandler::plan(const PlanningGoal &goal)
{
    if (std::holds_alternative<std::monostate>(goal))
    {
        ROS_ERROR("Planning goal is invalid.");
        ros::shutdown();
        return false;
    }
    if (std::holds_alternative<complete_planning_msgs::CartesianPlanGoal>(goal))
    {
        complete_planning_msgs::CartesianPlanGoal cartesian_goal = std::get<complete_planning_msgs::CartesianPlanGoal>(goal);
        cartesian_client_->sendGoal(cartesian_goal);
        cartesian_client_->waitForResult();

        if (cartesian_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Cartesian plan succeeded.");
            complete_planning_msgs::CartesianPlanResultConstPtr plan_result = cartesian_client_->getResult();
            trajectory_msgs::JointTrajectory planned_joint_traj =
                plan_result->planned_trajectory.joint_trajectory;
            std::pair<std::string, std::vector<double>> last_planned_configuration;
            last_planned_configuration.first = cartesian_goal.planning_group;
            last_planned_configuration.second =
                planned_joint_traj.points.back().positions;
            last_planned_configuration_ =
                std::make_unique<std::pair<std::string, std::vector<double>>>(
                    last_planned_configuration);
            complete_planning_msgs::ExecutePlanGoal execute_goal;
            execute_goal.motion_plan = plan_result->planned_trajectory;
            execute_goal.move_group_name = cartesian_goal.planning_group;
            execute_goal_queue.push(execute_goal);
        }
        else
        {
            ROS_ERROR("Cartesian plan failed.");
            return false;
        }
    }
    else if (std::holds_alternative<complete_planning_msgs::SlerpPlanGoal>(goal))
    {
        complete_planning_msgs::SlerpPlanGoal slerp_goal = std::get<complete_planning_msgs::SlerpPlanGoal>(goal);
        slerp_client_->sendGoal(slerp_goal);
        slerp_client_->waitForResult();

        if (slerp_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Slerp plan succeeded.");
            complete_planning_msgs::SlerpPlanResultConstPtr plan_result = slerp_client_->getResult();
            trajectory_msgs::JointTrajectory planned_joint_traj =
                plan_result->planned_trajectory.joint_trajectory;
            std::pair<std::string, std::vector<double>> last_planned_configuration;
            last_planned_configuration.first = slerp_goal.planning_group;
            last_planned_configuration.second =
                planned_joint_traj.points.back().positions;
            last_planned_configuration_ =
                std::make_unique<std::pair<std::string, std::vector<double>>>(
                    last_planned_configuration);
            complete_planning_msgs::ExecutePlanGoal execute_goal;
            execute_goal.motion_plan = plan_result->planned_trajectory;
            execute_goal.move_group_name = slerp_goal.planning_group;
            execute_goal_queue.push(execute_goal);
        }
        else
        {
            ROS_ERROR("Slerp plan failed.");
            return false;
        }
    }
    else if (std::holds_alternative<complete_planning_msgs::JointPlanGoal>(goal))
    {
        complete_planning_msgs::JointPlanGoal joint_goal = std::get<complete_planning_msgs::JointPlanGoal>(goal);
        joint_client_->sendGoal(joint_goal);
        joint_client_->waitForResult();

        if (joint_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Joint plan succeeded.");
            complete_planning_msgs::JointPlanResultConstPtr plan_result = joint_client_->getResult();
            trajectory_msgs::JointTrajectory planned_joint_traj =
                plan_result->planned_trajectory.joint_trajectory;
            std::pair<std::string, std::vector<double>> last_planned_configuration;
            last_planned_configuration.first = joint_goal.planning_group;
            last_planned_configuration.second =
                planned_joint_traj.points.back().positions;
            last_planned_configuration_ =
                std::make_unique<std::pair<std::string, std::vector<double>>>(
                    last_planned_configuration);
            complete_planning_msgs::ExecutePlanGoal execute_goal;
            execute_goal.motion_plan = plan_result->planned_trajectory;
            execute_goal.move_group_name = joint_goal.planning_group;
            execute_goal_queue.push(execute_goal);
        }
        else
        {
            ROS_ERROR("Joint plan failed.");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Unexpected Plan TYPE!");
        return false;
    }
    return true;
}

void TaskHandler::stopExecutionThread()
{
    run_thread = false;
    if (execution_thread_.joinable())
    {
        execution_thread_.join();
    }
}

bool TaskHandler::isExecutionQueueEmpty()
{
    return execute_goal_queue.empty();
}

bool TaskHandler::isExecutionThreadRunning()
{
    return run_thread;
}

void TaskHandler::handleExecution()
{
    while (ros::ok() && run_thread)
    {
        if (!execute_goal_queue.empty())
        {
            if (is_executing_)
            {
                execute_client_->waitForResult();
                if (execute_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    execute_goal_queue.pop();
                    if (execute_goal_queue.empty())
                    {
                        is_executing_ = false;
                        continue;
                    }
                    complete_planning_msgs::ExecutePlanGoal execute_goal = execute_goal_queue.front();
                    execute_client_->sendGoal(execute_goal);
                }
                else
                {
                    execute_goal_queue.pop();
                    ROS_ERROR("Execution failed.");
                    run_thread = false;
                }
            }
            else
            {
                std::cout << "Executing Trajectory" << std::endl;
                complete_planning_msgs::ExecutePlanGoal execute_goal = execute_goal_queue.front();
                execute_client_->sendGoal(execute_goal);
                is_executing_ = true;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    std::cout << "Killing the thread" << std::endl;
}
