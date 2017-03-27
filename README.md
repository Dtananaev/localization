Localization
======================================
Robot localization with deep neural networks on 2D occupancy grid maps.
The goal of the research project is to explore the capabilities of the neural networks to localize the robot on a 2D plane given the odometry and 2D laser scans.
The full report could be found here: https://drive.google.com/open?id=0B0jDQTJWpzD3ZnBHMDY1d0twU3c

Installation
======================================
1. Install ROS (http://www.ros.org/) 
2. Install Theano (http://deeplearning.net/software/theano/) or Tensorflow (https://www.tensorflow.org/) 
3. Install Keras (https://keras.io/)
4. Clone this repository into your ROS workspace
5. Clone navigation stack repository (http://wiki.ros.org/navigation) into the ROS workspace
6. Compile ROS packages: catkin_make -j1
7. You are done!


[![Build Status](https://travis-ci.org/Dtananaev/localization.svg?branch=master)](https://travis-ci.org/Dtananaev/localization)
[![BSD2 License](http://img.shields.io/badge/license-BSD2-brightgreen.svg)](https://github.com/Dtananaev/localization/blob/master/LICENSE.md) 

This repository contains:

* simulator - simulates the robot's noisy odometry, front and back 2D laser scans. It also outputs the true pose of the robot. 
The simulator could be used in order to generate the dataset for training of the neural network.
 [![simulator](https://github.com/Dtananaev/localization/blob/master/pictures/simulator.JPG)](https://www.youtube.com/watch?v=XgUfoiTanBc)
     * To build use: catkin_make -j1 
     * To run: 
        * roslaunch simulator simulator.launch - run simulator.
        * roslaunch simulator mbDWA.launch - run move_base - robot motion library. In order to move the robot use the button "2D nav goal"
* Scan_matcher - the neural network which uses only 2D laser scans in order to get the x,y,theta triplet representing the pose of a robot. In the video below the green marker is the robot's ground truth pose and the red marker is predicted by the network. 
 [![Scan_matcher](https://github.com/Dtananaev/localization/blob/master/pictures/scan.JPG)](https://www.youtube.com/watch?v=LuZNLaJ75xs)
     * To build use: catkin_make -j1 
     * To run: 
         * cd scan_matcher/scripts/
         * python scan_mather.py - this command runs the network online in the simulator.
         * python scan_matcher_train.py - this command runs the network in an online training regime. The network accumulates the data for 10 seconds, than makes one epoch of training and updates the weights on the fly. 
         
* recording tools - the tools for recording the simulated odometry, laser scans and true pose from simulator to txt files. It also contains the tools for visualization of the recorded data.         
* correction_net - the correction network is a network which is based on the idea that if we have a perfect odometry we have solved the localization problem. Since we cannot have the ideal odometry we can learn the noise distribution. So the idea of the correction network is to correct the error produced by the odometry through the use of the laser scan information. This is similar to the standard Kalman filter approach (for the details see [the full report](https://drive.google.com/open?id=0B0jDQTJWpzD3ZnBHMDY1d0twU3c)). 

The correction net architecture is based on the modified version of the VGG network implemented so far only for processing recorded data files in offline settings. The scripts for training and testing are provided. In the video below the green contour is the ground truth, the red contour is the corrected position from the network, and the blue contour is the odometry position.
 [![correction_net](https://github.com/Dtananaev/localization/blob/master/pictures/correction_net.JPG)](https://youtu.be/ULN8vkq5_bk)

* A bit of fun:). Another interesting application of the neural network is to represent the whole map inside the neural network's weights. So in the video below it is possible to see how a simple 4 layers fully connected network could output 181-dimensional vector of distance measurements given only x,y,theta as input.
[![regenerate_map](https://github.com/Dtananaev/localization/blob/master/pictures/laserGen.JPG)](https://www.youtube.com/watch?v=DWMxrn6dcgA)




