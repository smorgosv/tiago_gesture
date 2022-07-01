cmake_minimum_required(VERSION 2.8.3)
project(tiago_gesture)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  geometry_msgs
)

catkin_package(
)


include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(play_rps src/play_rps.cpp src/gestures.cpp)
add_dependencies(play_rps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(play_rps yaml-cpp
   ${catkin_LIBRARIES}
 )

add_executable(plan_gesture src/plan_gesture.cpp src/gestures.cpp)
add_dependencies(plan_gesture ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_gesture yaml-cpp
   ${catkin_LIBRARIES}
 )

install(TARGETS plan_gesture play_rps
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
