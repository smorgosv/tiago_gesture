#ifndef GESTURES_H
#define GESTURES_H

#include <moveit/move_group_interface/move_group_interface.h>

class Gesture{
    
  public:
    //Variables
    std::map<std::string, std::vector<std::vector<double>>> gestures;
    std::string gesture;
    std::vector<std::vector<double>> start_state;
    std::string planner_id;
    
    //Functions
    std::vector<std::vector<double>> getWaypoint(int point);
    std::vector<std::vector<double>> getStartState();
    int getGestureSize();
    void setGesture(std::string choice);
    void setStartState(std::vector<double> grip_start, std::vector<double> arm_start);
    void setPlannerId(std::string pid);
    void planToWaypoint(moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> &target, moveit::planning_interface::MoveGroupInterface::Plan &movement_plan);
    void planCurrentGesture();
    
    //Constructor & Destructor
    Gesture(std::string yaml_file);
    ~Gesture();
};

#endif
