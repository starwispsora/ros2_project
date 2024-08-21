20Aug
<br>
<br>

AM
##### made 1st bot launchers(tb3_0)
 - launch/hls_laser.launch.py
 - launch/my_tb3_bringup.launch.py
   
##### made 1st bot param(tb3_0)
 - burger.yaml

PM
##### build, run
enter : ssh ubuntu@192.168.14.20
build : 
run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0

<br>
<br>
21Aug
<br>
<br>

AM
#### made 2nd bot launchers(tb_1)
 - launch/hls_laser.launch.py
 - launch/my_tb3_bringup.launch.py

#### made 2nd bot param
 - burger.yaml

PM
##### build, run(tb_1)
enter : ssh ubuntu@192.168.14.14
build :
run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0

