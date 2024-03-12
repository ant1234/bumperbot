commands : 
start a robot state publisher node and send a xacro urdf model to it 
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro ~/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"

run a gui for moving joints 
ros2 run joint_state_publisher_gui joint_state_publisher_gui

run rviz2 graphical interface 
ros2 run rviz2
