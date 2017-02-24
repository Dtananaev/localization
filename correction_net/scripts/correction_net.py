#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
    
import tf.transformations
import tf
#import laser cast part
from Laser import Laser, Map
import math

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


#LOAD NETWORK
netname="CorrectionNet"
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
predicted_scan= LaserScan()
#ROS PART
init=False
#lasers
sensor_laser_callback=LaserScan()
sensor_laser= LaserScan()
odom=Odometry()
init_pose=[0,0,0]
#odometry
odom_previous=[0,0,0]
odom_current=[0,0,0]
#mapg
gridMap=OccupancyGrid()


def get_data():
    #odom update
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    odom_current[0] =copy(odom.pose.pose.position.x)
    odom_current[1]=copy(odom.pose.pose.position.y)
    odom_current[2]=euler[2]
    #laser_scan 
    sensor_laser=copy(sensor_laser_callback)

def normalize_angle(theta):
#Normalize phi to be between -pi and pi
    while theta> math.pi:
	    theta = theta - 2*math.pi

    while theta<-math.pi:
	    theta = theta + 2*math.pi

    return theta


def transform2map(odom, init_pose):
    x=odom[0]*np.cos(init_pose[2])-odom[1]*np.sin(init_pose[2])+init_pose[0]
    y=odom[0]*np.sin(init_pose[2])+odom[1]*np.cos(init_pose[2])+init_pose[1]
    theta=normalize_angle(init_pose[2]+odom[2])
    odom[0]=x
    odom[1]=y
    odom[2]=theta
    return odom

#callback functions
#Front scans
def callback_front(data):
    global sensor_laser
    sensor_laser=data



def callback_initpose(data):
    global init_pose
    global init
    print("I get init pose")
    init_pose[0]=copy(data.pose.pose.position.x)
    init_pose[1]=copy(data.pose.pose.position.y)
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    init_pose[2]=copy(euler[2])
    init=False

#Odometry
def callback_odom(data):
    global odom
    odom=data


def mapWrapper(occ_map, width,height):
    #print("occ_map",occ_map)
    occ_map=np.array(occ_map)
    for i in range(occ_map.shape[0]):
        if occ_map[i] >= 0:
           occ_map[i] = 1.0-(occ_map[i]/100.0)
        else:
           occ_map[i] = 1
    occ_map.resize(height, width)
    occ_map = occ_map.T
    return occ_map

def initCaster(wrappedMap,mapInfo):
    offset_x =mapInfo.origin.position.x
    offset_y =mapInfo.origin.position.y
    resolution = mapInfo.resolution

    #Parameters of the laser
    max_range = 50.0
    no_of_beams = 181
    min_angle = -math.pi/2.0
    resolution_angle = math.pi/(no_of_beams-1)
    noise_variance = 0.0#perfect

    map_obj = Map(mapInfo.height, mapInfo.width, offset_x, offset_y, resolution, wrappedMap)
    laser = Laser(max_range, min_angle, resolution_angle, no_of_beams, noise_variance, map_obj)
    return laser

def getLaserCast(robot_pos_x,robot_pos_y,robot_theta, laser):
    #relative position of the laser scanner to the center of the robot
    laser_pos_x = 1.2
    laser_pos_y = 0.0
    laser_angle = 0.0 * (math.pi/180.0)

    sin_theta = math.sin(robot_theta)
    cos_theta = math.cos(robot_theta)
    x = robot_pos_x + laser_pos_x * cos_theta - laser_pos_y*sin_theta
    y = robot_pos_y + laser_pos_x * sin_theta + laser_pos_y*cos_theta
    theta = robot_theta + laser_angle
    theta=normalize_angle(theta)

    ranges = laser.scan(x,y,theta)
    return ranges


    

def callback_map(data):
    global gridMap
    gridMap=data
    

def correction():
    global  init
    global sensor_laser
    global odom_previous
    global odom_current
    global gridMap
    global predicted_scan
    global model

    pub = rospy.Publisher('corrected_pose', PoseStamped, queue_size=1)
    pub_scan = rospy.Publisher('simulated_scans', LaserScan, queue_size=1000)
    p=PoseStamped()

    rospy.init_node('correction_net', anonymous=True)
    #listener = tf.TransformListener()
    #subscribe
    rospy.Subscriber('scan_front_perfect', LaserScan, callback_front)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    rospy.Subscriber('odom', Odometry, callback_odom)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, callback_initpose)
    rate = rospy.Rate(200) # 10hz
    rospy.sleep(1.)
    #getting map for scan cast
    wrappedMap= mapWrapper(gridMap.data,gridMap.info.width, gridMap.info.height)
    laser=initCaster(wrappedMap, gridMap.info)

    time_last_update = rospy.Time.now()
    update_weights_freq= rospy.Duration(10).to_sec()
    init_net=os.path.getmtime(weights)
    while not rospy.is_shutdown():
        if init==False:
            p.header.frame_id = "/map"
            p.header.stamp=rospy.Time(0)
            p.pose.position.x=init_pose[0]
            p.pose.position.y=init_pose[1]
            theta=init_pose[2]
            quat=tf.transformations.quaternion_from_euler(0,0,theta)
            p.pose.orientation.x=quat[0]
            p.pose.orientation.y=quat[1]
            p.pose.orientation.z=quat[2]
            p.pose.orientation.w=quat[3] 
            pub.publish(p) 
            print("init x", p.pose.position.x)
            print("init y", p.pose.position.y)    
            print("init theta", theta)    
            odom_previous[0]=copy(odom_current[0])
            odom_previous[1]=copy(odom_current[1])
            odom_previous[2]=copy(odom_current[2])
            #odom_previous=transform2map(odom_previous,init_pose)
            init=True
        if init==True:
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
            if sensor_laser.ranges!=[] : 
                #convert quat to theta
                get_data()
                #odom_current=transform2map(odom_current,init_pose)
                quaternion = (p.pose.orientation.x,
                              p.pose.orientation.y,
                              p.pose.orientation.z,
                              p.pose.orientation.w)

                euler = tf.transformations.euler_from_quaternion(quaternion)
                #generate scan for current pose +  delta odom
                
                #print('----------------------------------------------')
                #print("diff1 x", odom_current[0]-odom_previous[0])
                #print("diff1 y", odom_current[1]-odom_previous[1])    
                #print("diff1 theta", odom_current[2]-odom_previous[2]) 
                #print('----------------------------------------------') 
                updated_x=p.pose.position.x+odom_current[0]-odom_previous[0]
                updated_y=p.pose.position.y+odom_current[1]-odom_previous[1]
                updated_theta=euler[2]+odom_current[2]-odom_previous[2]
                updated_theta=normalize_angle(updated_theta)
                predicted_laser=getLaserCast( updated_x,updated_y,updated_theta,laser)


                max_range = 50.0
                no_of_beams = 181
                min_angle = -math.pi/2.0
                resolution_angle = math.pi/(no_of_beams-1)
                noise_variance = 0.0#perfect
                predicted_scan.angle_increment=resolution_angle
                predicted_scan.header.frame_id = "scan_front_frame"
                predicted_scan.header.stamp= rospy.Time(0)
                predicted_scan.angle_min=min_angle
                predicted_scan.angle_max= min_angle + resolution_angle+ no_of_beams-1;
                predicted_scan.ranges=np.array(predicted_laser)
                predicted_scan.range_min = 0
                predicted_scan.range_max = 50.0
                predicted_scan.scan_time = 0
                predicted_scan.time_increment = 0
                #print("predicted_laser",predicted_laser)
                pub_scan.publish(predicted_scan)
                data = np.hstack(( predicted_laser,  sensor_laser.ranges))
                print('data', data.shape)
                X_test=np.reshape(data , (1, 1, 1, 362))
                #print('X_test', X_test.shape)
                predicted_output = model.predict(X_test, batch_size=1)
                #print('----------------------------------------------')
                #print('predicted output x ', predicted_output[0][0])
                #print('predicted output y ', predicted_output[0][1])
                #print('predicted output theta ', predicted_output[0][2])
                #print('----------------------------------------------')
                p.header.frame_id = "map"
                
            
                delta_x=odom_current[0]-odom_previous[0]+ predicted_output[0][0] 
                delta_y=odom_current[1]-odom_previous[1] + predicted_output[0][1]
                #delta_x=odom_current[0]-odom_previous[0]
                #delta_y=odom_current[1]-odom_previous[1] 
                correction_x=delta_x*np.cos(init_pose[2])-delta_y*np.sin(init_pose[2])
                correction_y=delta_x*np.sin(init_pose[2])+delta_y*np.cos(init_pose[2])
                p.pose.position.x= p.pose.position.x+ correction_x
                p.pose.position.y=p.pose.position.y+ correction_y 
                theta= euler[2]+odom_current[2]-odom_previous[2]+predicted_output[0][2]
                #theta= euler[2]+odom_current[2]-odom_previous[2]
                theta =  normalize_angle(theta)
                #convert to quaterion
                theta =  normalize_angle(theta)
                quat=tf.transformations.quaternion_from_euler(0,0,theta)
                p.pose.orientation.x=quat[0]
                p.pose.orientation.y=quat[1]
                p.pose.orientation.z=quat[2]
                p.pose.orientation.w=quat[3] 
                pub.publish(p)
                #print('----------------------------------------------')
                #print("diff x", odom_current[0]-odom_previous[0])
                #print("diff y", odom_current[1]-odom_previous[1])    
                #print("diff theta", odom_current[2]-odom_previous[2]) 
                #print('----------------------------------------------') 
                odom_previous[0]=copy(odom_current[0])
                odom_previous[1]=copy(odom_current[1])
                odom_previous[2]=copy(odom_current[2])
                rate.sleep()
    rospy.spin()

if __name__ == '__main__':
     print('Start correction network')
     correction()



