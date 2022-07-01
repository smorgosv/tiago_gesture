#include <string>
#include <vector>
#include <map>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

// YAML Support
#include <yaml-cpp/yaml.h>
#include "gestures.h"

Gesture::Gesture(std::string yaml_file){ //Constructor

  // Sets default planner
  planner_id = "SBLkConfigDefault";
  
  // Load the given gestures yaml file and retrieves the root node play_gesture and the node containing all the gestures "gestures"
  YAML::Node node;
  node = YAML::LoadFile(yaml_file);
  YAML::Node node_play_gesture = node["play_gesture"];
  YAML::Node node_gestures = node_play_gesture["gestures"];

  // Constructs a map of gestures from the data in the given YAML file. Each gesture will have 2 entries into the map, one for the arm_torso move group and one for the gripper move group. They are denoted by [gesture]_gripper or [gesture]_arm.
  for(YAML::const_iterator it=node_gestures.begin(); it!= node_gestures.end(); it++)
  {
    std::string gesture = it->first.as<std::string>();
    std::string grip = gesture;
    std::string arm = gesture;
    grip.append("_gripper");
    arm.append("_arm");

    // Each gesture contains an arm, gripper and meta section
    YAML::Node node_gesture = node_gestures[gesture];
    YAML::Node arm_positions = node_gesture["arm"]; // Contains the waypoints for arm movements of the gesture
    YAML::Node gripper_positions = node_gesture["gripper"]; // Contains the waypoints for gripper movements of the gesture
    YAML::Node node_meta = node_gesture["meta"]; // Contains metadata about the gesture, currently unused

    // Goes through the arm waypoints in the gesture, pushes each waypoint as a vector to the map key of <gesture name>_arm
    for (std::size_t i=0;i<arm_positions.size();i++) {
      if (arm_positions[i]["positions"].IsSequence()) {
        std::vector<double> vi = arm_positions[i]["positions"].as<std::vector<double>>();

        ROS_DEBUG_STREAM("IsSequence: ");
        for (auto p: vi) {
          ROS_DEBUG_STREAM(p << " ,");
        }
        ROS_DEBUG_STREAM("\n");
        gestures[arm].push_back(vi);
      }
    }
    
    // Goes through the gripper waypoints in the gesture, pushes each waypoint as a vector to the map key of <gesture name>_gripper
    for (std::size_t i=0;i<gripper_positions.size();i++) {
      if (gripper_positions[i]["positions"].IsSequence()) {
        std::vector<double> vi = gripper_positions[i]["positions"].as<std::vector<double>>();
/*
        std::cout << "IsSequence: ";
        for (auto p: vi) {
          std::cout << p << " ,";
        }
        std::cout << "adding " << grip << std::endl;
*/
        gestures[grip].push_back(vi);
      }
    }
  }

// Debug prints the map

/*
  std::cout << "\n" << "These are the motions loaded for the gestures\n";

  for(const auto &g: gestures) {
    std::cout<< g.first << std::endl;
    for (auto &p: g.second) {
      std::cout << "{ " ;
      for (auto v: p) {
        std::cout << v << ", ";
      }
      std::cout << "}, " ;
    }
    std::cout<< std::endl;
  }
*/

}
    
Gesture::~Gesture(){ //Destructor

}
   
std::vector<std::vector<double>> Gesture::getWaypoint(int point){
  std::string grip = gesture;
  std::string arm = gesture;
  grip.append("_gripper");
  arm.append("_arm");
  return { gestures[grip][point], gestures[arm][point]};
}

void Gesture::setPlannerId(std::string pid) {
    planner_id = pid;
}

std::vector<std::vector<double>> Gesture::getStartState(){
  return start_state;
}

// Returns the amount of waypoints in the current gesture
int Gesture::getGestureSize(){
  std::string arm = gesture;
  arm.append("_arm"); // arbitrary lookup, arm and gripper gestures must be the same size
  return gestures[arm].size();
}
    
// Sets the gesture to be executed  
void Gesture::setGesture(std::string choice){

  gesture = choice;

}

// Sets the initial state of the robot's joints
void Gesture::setStartState(std::vector<double> grip_start, std::vector<double> arm_start){
  grip_start.insert(grip_start.begin(), 0); // Gripper joint 1 does not appear when values are taken from the gripper joints so a default value of zero is set
  arm_start.push_back(0); // Arm tool joint does not appear when values are taken from the arm joints so a default value of zero is set
  start_state = {grip_start, arm_start};

}

// Plans the trajectory the arm will take from its current state to the stated waypoint in the gesture
void Gesture::planToWaypoint(moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> &target, moveit::planning_interface::MoveGroupInterface::Plan &movement_plan){

  std::vector<std::string> joint_names = move_group.getJoints();
  
  // Ensures that the waypoint has the correct number of values for the move group selected
  if(target.size() > joint_names.size()){
    ROS_INFO_STREAM("\t" << "Waypoint has too many values. There sould be " << joint_names.size() << " values in the waypoint.");
    throw std::runtime_error("Too many waypoint values.");
  } else if (target.size() < joint_names.size()){
    ROS_INFO_STREAM("\t" << "Waypoint has too few values. There sould be " << joint_names.size() << " values in the waypoint.");
    throw std::runtime_error("Too few waypoint values.");
  }
  
  // Sets the targets of each joint
  for(unsigned int i = 0; i < target.size(); ++i){
  
    ROS_INFO_STREAM("\t" << joint_names[i] << " target position: " << target[i]);
    move_group.setJointValueTarget(joint_names[i], target[i]);
  
  }

  // Plans the trajectory
  move_group.setPlanningTime(5.0);
  bool success = bool(move_group.plan(movement_plan));

  // Throws if a plan is not found
  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << movement_plan.planning_time_ << " seconds");

}

/*
Planner IDs found at: https://github.com/pal-robotics/tiago_moveit_config/blob/kinetic-devel/config/ompl_planning.yaml

Valid Planner ID's from OMPL to try;
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
*/

void Gesture::planCurrentGesture() {

  // Creates move group interfaces for the arm and torso and the gripper as they are separate movegroups and there is no integrated one.
  moveit::planning_interface::MoveGroupInterface arm_torso_group("arm_torso");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  
  // States which planner will be used 
  arm_torso_group.setPlannerId(planner_id);
  
  // Sets the starting states of the move groups
  arm_torso_group.setStartStateToCurrentState();
  gripper_group.setStartStateToCurrentState();
  
  // Sets the scalling factors of velocity and accelaration to 1, lower values will make movement slower
  arm_torso_group.setMaxVelocityScalingFactor(1.0);
  gripper_group.setMaxVelocityScalingFactor(1.0);
  arm_torso_group.setMaxAccelerationScalingFactor(1.0);
  gripper_group.setMaxAccelerationScalingFactor(1.0);
  
  // Plans and moves to each of the gestures waypoints in the chosen gesture
  for (unsigned int i = 0; i < getGestureSize(); ++i){
    
    std::vector<std::vector<double>> waypoints = getWaypoint(i);
    
    // Creates plan objects for each move group
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    
    planToWaypoint(gripper_group, waypoints[0], gripper_plan); // Plan the gripper's movement
    planToWaypoint(arm_torso_group, waypoints[1], arm_plan); // Plan the arm's movement
   
    ros::Time start = ros::Time::now(); // Used to count full movement time

    moveit::core::MoveItErrorCode e1 = gripper_group.move(); // Move the gripper
    moveit::core::MoveItErrorCode e2 = arm_torso_group.move(); // Move the arm and torso
    if (!bool(e1))
      throw std::runtime_error("Error executing gripper plan");
    else if (!bool(e2))
      throw std::runtime_error("Error executing arm plan");

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
   
  }    
}
