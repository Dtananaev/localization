#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf.transformations
import tf
#import laser cast part
from Laser import Laser, Map
import math

from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation

from keras.optimizers import SGD
from copy import copy
import numpy as np
import os
from keras.models import model_from_json
from keras.callbacks import ModelCheckpoint
from keras.optimizers import Adam
optim=Adam(lr=0.00001, beta_1=0.9, beta_2=0.999, epsilon=1e-08)

#Network parameters
data_dim =362  
output_size=3 
batch_size=128
epoch=1

def normalize_angle(theta):
#Normalize phi to be between -pi and pi
    while theta> math.pi:
	    theta = theta - 2*math.pi

    while theta<-math.pi:
	    theta = theta + 2*math.pi

    return theta

def calculateScans(train_pose,delta_odom,laser):
    global data_laser_simul
    data_laser_simul=[]
    train_pose.pop()
    for i in range(len(train_pose)):
        update=train_pose[i]+delta_odom[i]
        predicted_laser=getLaserCast( update[0],update[1],update[2],laser)
        data_laser_simul.append(predicted_laser)

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




gridMap=OccupancyGrid()
callback_front_laser=[]
callback_pose_new=PoseStamped()
odom=Odometry()
#training data
data_pose=[]
data_laser=[]
data_laser_simul=[]
data_odom=[]

def get_data():
    global callback_front_laser
    global callback_pose_new
    global data_pose
    global data_laser
    global odom
    global data_odom

    # get pose
    temp=[None]*3
    temp[0]=copy(callback_pose_new.pose.position.x)
    temp[1]=copy(callback_pose_new.pose.position.y)
    quaternion = (
    callback_pose_new.pose.orientation.x,
    callback_pose_new.pose.orientation.y,
    callback_pose_new.pose.orientation.z,
    callback_pose_new.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    temp[2]=copy(euler[2])
    data_pose.append(copy(temp))
    #get laser front
    data_laser.append(copy(callback_front_laser))

    #odom update
    odom_current=[None]*3
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    odom_current[0] =copy(odom.pose.pose.position.x)
    odom_current[1]=copy(odom.pose.pose.position.y)
    odom_current[2]=euler[2]
    data_odom.append(odom_current)

def callback_map(data):
    global gridMap
    gridMap=data
    

def callback_front(data):
    global callback_front_laser
    callback_front_laser=data.ranges



def callback_pose(data):
    global callback_pose_new
    callback_pose_new=data
    
    
def callback_odom(data):
    global odom
    odom=data


def both():
    global data_laser
    global data_laser_simul
    global data_pose
    global odom
    global gridMap
    
    rospy.init_node('correction_net_train', anonymous=True)
    #subscribe
    rospy.Subscriber('scan_front', LaserScan, callback_front)
    rospy.Subscriber('true_pose', PoseStamped, callback_pose)
    rospy.Subscriber('odom', Odometry, callback_odom)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(2.)
    
    
    time_last_update = rospy.Time.now()
    update_weights_freq= rospy.Duration(10).to_sec()

    wrappedMap= mapWrapper(gridMap.data,gridMap.info.width, gridMap.info.height)
    laser=initCaster(wrappedMap, gridMap.info)
    while not rospy.is_shutdown():
        rate.sleep()
        diff_time=rospy.Time.now()-time_last_update# frequency of update
        get_data()
        if(diff_time.to_sec()>=update_weights_freq):
            time_last_update=rospy.Time.now()
            train_front=copy(data_laser)
            train_pose=copy(data_pose)
            train_odom=copy(data_odom)
            #generate laser scans from odom position

          
            delta_odom=np.diff(train_odom,axis=0)
            delta_pose=np.diff(train_pose,axis=0)
            print("delta_odom",delta_odom.shape)
            print("delta_pose",delta_pose.shape)
            train_front.pop(0)

            calculateScans(train_pose,delta_odom,laser)
            delta=delta_pose-delta_odom

            print("train_front", len(train_front))
            print("train_simul", len(data_laser_simul))

            data=np.hstack((data_laser_simul,train_front))
            X_train=np.reshape(data, (data.shape[0],1,1, data_dim))
            print('X_train',X_train.shape)
            y_train=np.array( delta)
            print('y_train',y_train.shape)
            netname="CorrectionNet"
            if not os.path.isfile(netname+'.h5') or not os.path.isfile(netname+'.json'):
                print('NO NETWORK')
            else:
                print('Loading existing network')
                model = model_from_json(open(netname+'.json').read())
                model.load_weights(netname+'.h5')

                #compile loaded model
                model.compile(loss='mse',optimizer=optim)
                print('Start learning...')
                model.fit(X_train, y_train, nb_epoch=epoch,batch_size=batch_size)
                model.save_weights(netname+'.h5', overwrite=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     print('Start retraining')
     both()







