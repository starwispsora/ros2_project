20Aug
<br>
<br>

AM Sora
##### made 1st bot launchers(tb3_0)
 - launch/hls_laser.launch.py
 - launch/my_tb3_bringup.launch.py
   
##### made 1st bot param(tb3_0)
 - burger.yaml

PM Suengyeon
##### build, run
enter : ssh ubuntu@192.168.14.14
build : cd robot_ws/
cbp my_tb3_launcher
run : 
in turtle run : ros2 launch my_tb3_launcher my_tb3_bringup.launch.py
in laptop run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0

<br>
<br>
21Aug
<br>
<br>

AM Sora
#### made 2nd bot launchers(tb3_1)
 - launch/hls_laser.launch.py
 - launch/my_tb3_bringup.launch.py

#### made 2nd bot param(tb3_1)
 - burger.yaml

##### build, run(tb3_1)
enter : ssh ubuntu@192.168.14.13
build : cd robot_ws/
cbp my_tb3_launcher
run : 
in turtle run : ros2 launch my_tb3_launcher my_tb3_bringup.launch.py
in laptop run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_1

PM
Suengyoen
 - build 2 cpp files and integrate them into one node
 - write camera recognition code
