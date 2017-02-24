#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf.transformations
import tf
#import Keras part
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation

from keras.optimizers import SGD
from copy import copy
import numpy as np
import os
from keras.models import model_from_json
from keras.callbacks import ModelCheckpoint
import math
import os.path, time

#LOAD NETWORK
netname="ScanMatchin"
if not os.path.isfile(netname+'.h5') or not os.path.isfile(netname+'.json'):
   print('NO NETWORK')
else:
    print('Loading existing network')
    model = model_from_json(open(netname+'.json').read())
    model.load_weights(netname+'.h5')

    #compile loaded model
    model.compile(loss='mse',
                  optimizer='adam')


#retrained weights
weights= netname+'.h5'


    

#ROS PART
front_laser= []
back_laser= []

def normalize_angle(theta):
#Normalize phi to be between -pi and pi

    while theta> math.pi:
	    theta = theta - 2*math.pi

    while theta<-math.pi:
	    theta = theta + 2*math.pi

    return theta



def callback_front(data):
    global front_laser
    front_laser =copy(data.ranges)


def callback_back(data):
    global back_laser
    back_laser=copy(data.ranges)

def both():
    global front_laser
    global back_laser
    global model
    pub = rospy.Publisher('scan_matcher_pose', PoseStamped, queue_size=1)
    pub_contour = rospy.Publisher('scan_matcher_contour', Marker, queue_size=1)
    rospy.init_node('scan_matcher', anonymous=True)
    #subscribe
    rospy.Subscriber('scan_front', LaserScan, callback_front)
    rospy.Subscriber('scan_rear', LaserScan, callback_back)
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(2.)
    time_last_update = rospy.Time.now()
    update_weights_freq= rospy.Duration(10).to_sec()
    #check the time of the last update of the weights
    init_net=os.path.getmtime(weights)

    while not rospy.is_shutdown():
        #check the time untill update of the weights
        diff_time=rospy.Time.now()-time_last_update# frequency of update

        if(diff_time.to_sec()>=update_weights_freq and init_net!=os.path.getmtime(weights)):
            init_net=os.path.getmtime(weights)
            print('Loading retrained network')
            model = model_from_json(open(netname+'.json').read())
            model.load_weights(weights)
            #compile loaded model
            model.compile(loss='mse', optimizer='adam')
            time_last_update = rospy.Time.now()


           
  
        data = np.hstack((  back_laser,  front_laser))
        #print('data', data.shape)
        X_test=np.reshape(data , (1, 362))
        predicted_output = model.predict(X_test, batch_size=1)
        #print('predicted output x ', predicted_output[0][0])
        p=PoseStamped()
        p.header.frame_id = "/map";
        p.pose.position.x= predicted_output[0][0]
        p.pose.position.y= predicted_output[0][1]
        theta =  normalize_angle(predicted_output[0][2])
        quat=tf.transformations.quaternion_from_euler(0,0,theta)
        p.pose.orientation.z=quat[2]
        p.pose.orientation.w=quat[3] 
        pub.publish(p)   
        
    #
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        #rate.sleep()
    #pub contour
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp =rospy.Time(0)
        marker.id = 1
        marker.type =Marker.LINE_STRIP
        marker.action =Marker.ADD


        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
 

        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
    #theta =tf::getYaw(pose.pose.orientation);
    #double x,y;
        point1=Point()
    #point 1 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
        point1.x = np.cos(theta)* (-0.5) -np.sin(theta)*(-0.55) +p.pose.position.x
        point1.y = np.sin(theta)*(-0.5)+np.cos(theta)*(-0.55)+ p.pose.position.y
        marker.points.append(point1);
        point2=Point()
 #point 2 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
        point2.x = np.cos(theta)* (-0.5) -np.sin(theta)*(0.55) +p.pose.position.x
        point2.y = np.sin(theta)*(-0.5)+np.cos(theta)*(0.55)+ p.pose.position.y
        marker.points.append(point2);
        
        point3=Point()
 #point 3 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
        point3.x = np.cos(theta)* (1.75) -np.sin(theta)*(0.55) +p.pose.position.x
        point3.y = np.sin(theta)*(1.75)+np.cos(theta)*(0.55)+ p.pose.position.y
        marker.points.append(point3);

        point4=Point()
 #point 4 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
        point4.x = np.cos(theta)* (1.75) -np.sin(theta)*(-0.55) +p.pose.position.x
        point4.y = np.sin(theta)*(1.75)+np.cos(theta)*(-0.55)+ p.pose.position.y
        marker.points.append(point4);
        point5=Point()
 #close contour [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
        point5.x = np.cos(theta)* (-0.5) -np.sin(theta)*(-0.55) +p.pose.position.x
        point5.y = np.sin(theta)*(-0.5)+np.cos(theta)*(-0.55)+ p.pose.position.y
        marker.points.append(point5);

        pub_contour.publish(marker)
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     print('Start processing')
     both()
