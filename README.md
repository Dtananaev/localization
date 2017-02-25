Localization
======================================
Robot localization with Deep neural network on 2D occupancy grid maps.
The goal of the research project is to explore the capabilities of the neural networks to localize the robot on 2D plane given the odometry and 2D laser scans.
The full report could be found here:https://drive.google.com/open?id=0B0jDQTJWpzD3ZnBHMDY1d0twU3c


[![Build Status](https://travis-ci.org/Dtananaev/ROS_nodes.svg?branch=master)](https://travis-ci.org/Dtananaev/ROS_nodes)
[![BSD2 License](http://img.shields.io/badge/license-BSD2-brightgreen.svg)](https://github.com/Dtananaev/ROS_nodes/blob/master/LICENSE.md) 

It contains:

* graph_planner
 [![graph_planner](https://github.com/Dtananaev/ROS_nodes/blob/master/pictures/graph_planner.JPG)](https://www.youtube.com/watch?v=dBtdjbe6mtw&list=PLm9UBVQa6prlMgzgwN7DHSIl7TJIrmwM-)
     * To install use: catkin_make -j1 
     * To run: roslaunch graph_planner graph.launch
* multimaster_example
 [![multimaster_example](https://github.com/Dtananaev/ROS_nodes/blob/master/pictures/multimaster.JPG)](https://www.youtube.com/watch?v=59T0iIJnUGk&list=PLm9UBVQa6prlMgzgwN7DHSIl7TJIrmwM-&index=7)
     * To install use: catkin_make -j1 
     * To run: 
         * open launch file ../multimaster_example/launch/multimaster.launch  and add the ip of foreign PC and port of roscore 
         * run roscore on both PC (not necessary to run roscore on foreign PC before launching multimaster because it has autoconnect)
         * run talker node: rosrun multimaster_example talker
         * run multimaster: roslaunch multimaster_example multimaster.launch
     * Troubleshooting: 
         * Problem with sending messages between PCs, type on both PC (add to ~/.bashrc): export ROS_HOSTNAME=ip_adress_of_pc(e.g.export ROS_HOSTNAME=192.168.0.10 )   
* multimaster - the full version of the multimaster which allows to broadcast the list of topics and tf transforms from host pc to foreign pc. This list specified in the config.yaml file. Other properties specified in the launch file (see multimaster.launch). 
 [![multimaster](https://github.com/Dtananaev/ROS_nodes/blob/master/pictures/mm.JPG)](https://www.youtube.com/watch?v=VnkDEB2HQ4E&feature=youtu.be)
     * To install use: catkin_make -j1 
     * To run: 
         * open launch file ../multimaster/launch/multimaster.launch  and add the ip of foreign PC and port of roscore 
         * open the file multimaster/config.yaml  and specify the lists of topics that you want to broadcasts to foreign PC
         * run roscore on both PC (not necessary to run roscore on foreign PC before launching multimaster because it has autoconnect)
         * run multimaster: roslaunch multimaster multimaster.launch
     * Troubleshooting: 
         * Problem with sending messages between PCs, type on both PC (add to ~/.bashrc): export ROS_HOSTNAME=ip_adress_of_pc(e.g.export ROS_HOSTNAME=192.168.0.10 )  
