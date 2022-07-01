Make sure to have Ubuntu 18.04 installed and using ROS Melodic. Instructions on how to install that for the TIAGo can be found at the followiing address: http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS. 

Once the TIAGo's dependencies and software has been downloaded, make a catkin workspace and add the tiago_gesture folder into the directory catkin_ws/src/.

Then go back to the catkin workspace, do 'source devel/setup.bash' and then 'catkin_make' everything should compile and run now. 

Launch the simulation module by using the launch file included in the project.

roslaunch tiago_gesture doit.launch

once that is running, pull up a second terminal, the nodes can now be invoked.

Either:
rosrun tiago_gesture plan_gesture <gesture> [planner id]

Or:
rosrun tiago_gesture play_rps <rock|paper|scissors>
