#ifndef MISSION_MANAGER_HPP
#define MISSION_MANAGER_HPP

#include <string>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tuple>
#include <math.h>
#include "franka_custom_controllers/my_utils.hpp"

using namespace Eigen;
struct Tasks{
    std::vector<std::string> go_to{"R_M"};
    std::vector<std::string> teleop_move{"T_M"};
    std::vector<std::string> stop{};
};
struct Mission{
    int phase;
    int previous_phase;
    float phase_time;
    std::vector<std::string> prev_action, curr_action;
};


class MissionManager{

  private:
   
    Mission mission_data;
    Tasks tasks;
    bool containTask( const std::vector<std::string>& tasks, const std::string& task_name){
        return std::find(tasks.begin(), tasks.end(), task_name) != tasks.end();
    }
  public: 

    MissionManager() {
    };
    ~MissionManager()= default;
    void starting(){
        mission_data.phase = 0;
        mission_data.phase_time = 0;
        mission_data.prev_action = tasks.stop;
        mission_data.curr_action = tasks.stop;
    }
    void updateMissionData(){
        if (mission_data.phase ==0){
            mission_data.prev_action = mission_data.curr_action;
            mission_data.curr_action = tasks.stop;
        }
        if (mission_data.phase ==1){
            mission_data.prev_action = mission_data.curr_action;
            mission_data.curr_action = tasks.go_to;
        }
        if (mission_data.phase == 2){
            mission_data.prev_action = mission_data.curr_action;
            mission_data.curr_action = tasks.stop;
        }
        if (mission_data.phase == 3){
            mission_data.prev_action = mission_data.curr_action;
            mission_data.curr_action = tasks.teleop_move;
        }
    }
    double actionTransition(const std::string case_name, std::vector<std::string> prev_action, std::vector<std::string> curr_action, double time){
        //std::cout << "Prev action: ";
        for (const auto& action : prev_action){
            //std::cout << action << " ";
        }
        //std::cout << std::endl;
        //std::cout << "Curr action: ";
        for (const auto& action : curr_action){
            //std::cout << action << " ";
        }
        //std::cout << std::endl;
        //std::cout << "Case name: " << case_name << std::endl;
        if (containTask(prev_action, case_name) && containTask(curr_action, case_name)){
            return 1.0;
        }else if (containTask(curr_action, case_name) && !containTask(prev_action, case_name)){
            return increasingBellShapedFunction(0.0, 1.0, 0.0, 1.0,  time);
        }else if (containTask(prev_action, case_name) && !containTask(curr_action, case_name)){
            return decreasingBellShapedFunction(0.0, 1.0, 0.0, 1.0, time);
        }else if (!containTask(prev_action, case_name) && !containTask(curr_action, case_name)){
            return 0.0;
        }
        return 0.0;
    }
    std::vector<std::string> getPrevAction(){
        return mission_data.prev_action;
    }
    std::vector<std::string> getCurrAction(){
        return mission_data.curr_action;
    }
    double getPhaseTime(){
        return mission_data.phase_time;
    }
    int getPhase(){
        return mission_data.phase;
    }
    int getPreviousPhase(){
        return mission_data.previous_phase;
    }
    void setPhase(int previous_phase,int phase){
        //ROS_WARN("Setting phase from %d to %d", mission_data.phase, phase);
        mission_data.phase = phase;
        mission_data.previous_phase = previous_phase;
        mission_data.phase_time = 0;
    }
    void setPhaseTime(double delta_phase_time){
        mission_data.phase_time += delta_phase_time;
    }

};

#endif // MISSION_MANAGER_HPP

