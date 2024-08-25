***
Project PPT(Korean) : https://docs.google.com/presentation/d/1ZqtcRpdbZTo8quWRLtGf3FaFsqSgyMQdPg0ZLWDysmE/edit#slide=id.g742e3e7cd_0_0

***

20Aug
<br>
<br>
AM Sora
##### made 1st bot launchers(tb3_0)
 - launch/hls_laser.launch.py
 - launch/my_tb3_bringup.launch.py
   
##### made 1st bot param(tb3_0)
 - burger.yaml

PM Seungyeon
##### build, run
enter : ssh ubuntu@192.168.14.14 <br>
build : cd robot_ws/ <br>
cbp my_tb3_launcher <br>
run : 
in turtle run : ros2 launch my_tb3_launcher my_tb3_bringup.launch.py <br>
in laptop run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0
<br>
<br>
***
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
enter : ssh ubuntu@192.168.14.13 <br>
build : cd robot_ws/ <br>
cbp my_tb3_launcher <br>
run : 
in turtle run : ros2 launch my_tb3_launcher my_tb3_bringup.launch.py <br>
in laptop run : ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_1

PM Seungyoen
 - build 2 cpp files and integrate them into one node
 - additionally, write and add opencv camera line coloer recognition code and quantization(=the process of mapping continuous infinite values to a smaller set of discrete finite values)
***
22Aug <br>

AM Sora
 - Completed the build and configuration for both TurtleBots (tb3_0 and tb3_1).
 - Finalized the launch files for both bots: <br>
        launch/hls_laser.launch.py
        launch/my_tb3_bringup.launch.py
 - Finalized the parameter files for both bots: <br>
        burger.yaml

PM Seungyeon
 - Successfully built the integrated node that combines the functionality of the two CPP files.
 - Tested and confirmed the integration of the OpenCV-based camera line color recognition and quantization code.
 - Completed the full system build and verified the operation of both TurtleBots with their respective nodes.
***
23Aug <br>

AM Sora & Seungyeon
 - Collaboratively wrote the project presentation and final report.
 - Documented the entire development process and outlined the key functionalities implemented in the TurtleBot project.
 - Prepared and finalized the slides for the upcoming presentation.

***
