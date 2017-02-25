Localization
======================================
Robot localization with Deep neural network on 2D occupancy grid maps.
The goal of the research project is to explore the capabilities of the neural networks to localize the robot on 2D plane given the odometry and 2D laser scans.
The full report could be found here:https://drive.google.com/open?id=0B0jDQTJWpzD3ZnBHMDY1d0twU3c

Installation
======================================
1. Install ROS (http://www.ros.org/) 
2. Install Theano(http://deeplearning.net/software/theano/) or Tensorflow(https://www.tensorflow.org/) 
3. Install Keras(https://keras.io/)
4. Clone this repository into ROS workspace
5. Clone navigation stack repository (http://wiki.ros.org/navigation) into ROS workspace
6. Compile ROS workspace: catkin_make -j1
7. You are done!

[![Build Status](https://travis-ci.org/Dtananaev/ROS_nodes.svg?branch=master)](https://travis-ci.org/Dtananaev/ROS_nodes)
[![BSD2 License](http://img.shields.io/badge/license-BSD2-brightgreen.svg)](https://github.com/Dtananaev/ROS_nodes/blob/master/LICENSE.md) 

It contains:

* simulator - simulate the robot  noisy odometry and front and back 2D laser scans. It also output the true pose of the robot. 
The simulator could be used in order to generate the dataset for neural network training.
 [![simulator](https://github.com/Dtananaev/localization/blob/master/pictures/simulator.JPG)](https://www.youtube.com/watch?v=XgUfoiTanBc)
     * To install use: catkin_make -j1 
     * To run: roslaunch simulator simulator.launch - run simulator 
               roslaunch simulator mbDWA.launch - run move_base- robot motion library. 
               In order to move the robot use the button "2D nav goal"
* Scan_matcher - the neural network which uses only 2D laser scans in order to get the x,y,theta pose of robot. In video below the green is the robot ground truth pose and red is predicted by network. 
 [![Scan_matcher](https://github.com/Dtananaev/localization/blob/master/pictures/scan.JPG)](https://www.youtube.com/watch?v=LuZNLaJ75xs)
     * To install use: catkin_make -j1 
     * To run: 
         * cd scan_matcher/scripts/
         * python scan_mather.py - this command run network online in simulator.
         * pyhon scan_matcher_train.py - this command run network in online training regime. The network accumulate the data for 10 seconds than make one epoch of training and updates the weights on the fly. 
         
* recording tools - the tools for recording the simulated data from simulator (odometry, laser scans, true pose) into txt files and also it contains the tools for visualization of the recorded data.         
* correction_net - the correction network is network based on the idea that if we have a perfect odometry we have solved localization problem. The idea of the correction network is to correct the error produced by the odometry by using laser scan information. THe idea based on standart Kalman filter approach (for the details see the full report: https://drive.google.com/open?id=0B0jDQTJWpzD3ZnBHMDY1d0twU3c). The correction net architecture based on modified version VGG network architecture implemented so far only for processing recorded data files in offline settings. The scripts for training and testing provided. 
 [![correction_net](https://github.com/Dtananaev/localization/blob/master/pictures/correction_net.JPG)](https://www.youtube.com/watch?v=VnkDEB2HQ4E&feature=youtu.be)
     * To install use: catkin_make -j1 
     * To run: 
         * open launch file ../multimaster/launch/multimaster.launch  and add the ip of foreign PC and port of roscore 
         * open the file multimaster/config.yaml  and specify the lists of topics that you want to broadcasts to foreign PC
         * run roscore on both PC (not necessary to run roscore on foreign PC before launching multimaster because it has autoconnect)
         * run multimaster: roslaunch multimaster multimaster.launch
     * Troubleshooting: 
         * Problem with sending messages between PCs, type on both PC (add to ~/.bashrc): export ROS_HOSTNAME=ip_adress_of_pc(e.g.export ROS_HOSTNAME=192.168.0.10 )  
