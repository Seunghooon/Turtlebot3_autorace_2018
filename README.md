# Turtlebot3_autorace_2018

## Robot

<img src="https://github.com/Seunghooon/turtlebot3_autorace_2018/blob/master/readme_images/bumblebee.png" width="500">

#### Hardware components
 - Raspberry pi3
 - HC-SR04 Ultrasonic Sensor
 - Logitech C930e USB Webcam
 - D-Link dwa-171(It was used to catch wifi 5G.) 
 
#### Software used
 - ROS Kinetic
 - Python 2
 - OpenCV 3
 - Arduino Firmware
 - Ubuntu 16.04(Remote PC, Raspberry pi3)

#### Run master Node
 - To execute master node commands are:
'''
 $ rosrun <package_name> main.py
'''

 - The code I tried to make 'main.py' is in the 'test_code' folder
#### Run turtlebot3 tunnel node
 - Bring up basic packages to start Turtlebot3 applications in raspberry pi3.
'''
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
'''
 - If you want to launch lidar sensor and core separately, please use below commands.

'''
$ roslaunch turtlebot3_bringup turtlebot3_lidar.launch
$ roslaunch turtlebot3_bringup turtlebot3_core.launch
'''

 - Execute turtlebot3 tunnel node.
'''
$ rosrun <package_name> in_tunnel1.py
$ rosrun <package_name> in_tunnel2.py
'''

#### 왕밤빵왕밤빵왕밤빵's official blog
 - Blog: [https://blog.naver.com/killerbee_]




