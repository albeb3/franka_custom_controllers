#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>

std_msgs::Float32 goal_width_;
bool open_commanded_ = false;
bool grasp_commanded_ = false;
bool stop_commanded_ = false;
bool stop_in_progress_ = false;

/*
void unity_callback_open(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Received command from Unity: %f", msg->data);
    goal_width_.data = msg->data;
}
*/
void set_flag(bool& flag){
    if(!flag){
         flag = true;
    }
    else ros::Duration(0.8).sleep(); // Debounce to avoid multiple triggers
}
void unity_callback_open(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("Received open command from Unity: %d", msg->data);
    set_flag(open_commanded_);
    
}

void unity_callback_grasp(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("Received grasp command from Unity: %d", msg->data);
    if (msg->data){
        set_flag(grasp_commanded_);
    }
    else{
        set_flag(stop_commanded_);
    }   
}
void unity_callback_release(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO("Received release command from Unity: %d", msg->data);
    if (msg->data){
        set_flag(stop_commanded_);
    }   
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "franka_gripper_move_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/gripper_open", 10, unity_callback_open);
    ros::Subscriber sub_grasp = nh.subscribe("/gripper_grasp", 10, unity_callback_grasp);
    

    // Action client
    actionlib::SimpleActionClient<franka_gripper::MoveAction> client(
        "/franka_gripper/move", true);
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client(
        "/franka_gripper/grasp", true);    
    
    ros::Rate rate(100.0);open_commanded_ = false;
    
    while (ros::ok()){
    if (open_commanded_)
    {
        ROS_INFO("Waiting for franka_gripper move action server...");
        client.waitForServer();
        ROS_INFO("Move Action server connected.");
        franka_gripper::MoveGoal open_goal;
        open_goal.width = 0.08; // Open gripper to maximum width
        open_goal.speed = 0.1; // m/s
        ROS_INFO("Sending gripper open goal: width=%.3f speed=%.3f",
                open_goal.width, open_goal.speed);
        client.sendGoal(open_goal);
        ros::Duration(0.8).sleep(); // Allow some time for the action to start
        if (!client.waitForResult(ros::Duration(5.0))) {
            ROS_WARN("Gripper move action did not finish before the time out.");
        } else {
            ROS_INFO("Gripper move action finished: %s", 
                     client.getState().toString().c_str());
            open_commanded_ = false;
        }
        
    }
    if (grasp_commanded_)
    {
        ROS_INFO("Waiting for franka_gripper grasp action server...");
        grasp_client.waitForServer();
        ROS_INFO("Grasp Action server connected.");
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.width = 0.3; // Close gripper
        grasp_goal.epsilon.inner = 0.005; // 5 mm
        grasp_goal.epsilon.outer = 0.005; // 5 mm
        grasp_goal.speed = 0.01; // m/s
        grasp_goal.force = 5.0; // N
        ROS_INFO("Sending gripper grasp goal: width=%.3f speed=%.3f force=%.3f",
                grasp_goal.width, grasp_goal.speed, grasp_goal.force);
        grasp_client.sendGoal(grasp_goal);
        ros::Duration(0.8).sleep(); // Allow some time for the action to start
        if (!grasp_client.waitForResult(ros::Duration(5.0))) {
            ROS_WARN("Gripper grasp action did not finish before the time out.");
        } else {
            ROS_INFO("Gripper grasp action finished: %s", 
                     grasp_client.getState().toString().c_str());
            grasp_commanded_ = false;
            stop_in_progress_ = true;
        }
        
    }
    if (stop_in_progress_ && stop_commanded_)
    {
        ROS_INFO("Stopping gripper action...");
        actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client(
            "/franka_gripper/stop", true);    
        stop_client.waitForServer();
        franka_gripper::StopGoal stop_goal;
        stop_client.sendGoal(stop_goal);
        stop_commanded_ = false;
    }
     ros::spinOnce();
    }
    return 0;
}
