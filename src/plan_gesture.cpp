#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <string>
#include <vector>
#include <map>
#include "gestures.h"




int main(int argc, char** argv)
{
  // Set Log Level to Info/Debug
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "plan_gesture");

  if ( argc < 2 || argc > 3 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_gesture plan_gesture <gesture> [planner]");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }
  
  std::string yaml_name = ros::package::getPath("tiago_gesture") + "/resources/gestures.yaml";
  
  Gesture choice(yaml_name);
  std::string gesture = argv[1];
  choice.setGesture(gesture);

  if (argc == 3) {
    choice.setPlannerId(argv[2]);
  }
  
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  choice.planCurrentGesture();

  
  spinner.stop();

  return EXIT_SUCCESS;
}


