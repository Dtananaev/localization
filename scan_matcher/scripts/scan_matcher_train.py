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



callback_front_laser=[]
callback_back_laser=[]
callback_pose_new=PoseStamped()
#training data
data_pose=[]
data_laser=[]
data_laser_back=[]

def get_data():
    global callback_front_laser
    global callback_back_laser
    global callback_pose_new
    global data_pose
    global data_laser
    global data_laser_back

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
    data_laser_back.append(copy(callback_back_laser))



def callback_front(data):
    global callback_front_laser
    callback_front_laser=data.ranges


def callback_back(data):
    global callback_back_laser
    callback_back_laser=data.ranges


def callback_pose(data):
    global callback_pose_new
    callback_pose_new=data
    
    



def both():
    global data_laser
    global data_laser_back
    global data_pose
    rospy.init_node('scan_matcher_train', anonymous=True)
    #subscribe
    rospy.Subscriber('scan_front', LaserScan, callback_front)
    rospy.Subscriber('scan_rear', LaserScan, callback_back)
    rospy.Subscriber('true_pose', PoseStamped, callback_pose)
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(2.)
    
    time_last_update = rospy.Time.now()
    update_weights_freq= rospy.Duration(10).to_sec()

    while not rospy.is_shutdown():
        rate.sleep()
        diff_time=rospy.Time.now()-time_last_update# frequency of update
        get_data()
        if(diff_time.to_sec()>=update_weights_freq):
            time_last_update=rospy.Time.now()
            train_front=copy(data_laser)
            train_back=copy(data_laser_back)
            train_pose=copy(data_pose)
            #data_laser=[]
            #data_laser_back=[]
            #data_pose=[]
            print("train_front", len(train_front))
            print("train_back", len(train_back))

            X_train=np.hstack((train_back,train_front))
            print('X_train',X_train.shape)
            y_train=np.array( train_pose)
            print('y_train',y_train.shape)
            netname="ScanMatchin"
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







