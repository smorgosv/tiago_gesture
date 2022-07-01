#roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel > gazebo.log 2>&1 &
roslaunch tiago_gesture doit.launch > gazebo.log 2>&1 &
