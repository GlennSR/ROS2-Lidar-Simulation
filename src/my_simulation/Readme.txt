
ros2 run teleop_twist_keyboard teleop_twist_keyboard
-- To run teleop keyboard control

ros2 launch urdf_tutorial display.launch.py model:<=/home/glenio/path_to_urdf>
-- Launch Rviz to simply visualize a urdf without having to create a package and a launch file

ros2 run tf2_tools view_frames
-- Creates a pdf cointaining all the links relation in a Rviz launched robot (Needs to have the Rviz opened to work*)

killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
-- kill all gazebo processes

ros2 launch my_simulation maquette_robot_launch.py --show-args
-- Shows launch file args like 'world' and 'model'

