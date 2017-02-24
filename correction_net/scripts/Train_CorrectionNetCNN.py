from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten

from keras.optimizers import SGD
from copy import copy
import numpy as np
import os
from keras.models import model_from_json
from keras.callbacks import ModelCheckpoint
from keras.optimizers import Adam
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D

#Network parameters
data_dim=362  # 181 front lasers 181 odometry lasers
output_size=3 # corrections
batch_size=128
epoch=1000

#
nb_channels =1 
nb_height = 1
nb_width = data_dim
#training data
absolute_laser=[]
odom_laser=[]
correction=[]

#load from file

#load absolute laser (front laser with sensor with noise)
with open('../data/data_train/absolute_laser.txt') as f:
    txt=f.read()
    lines=txt.split('\n')
    for line in lines:
        if line: absolute_laser.append( map( float, line.split(',') )  )

#load odom laser (front laser with sensor with noise simulated from odom position)        
with open('../data/data_train/odom_perfect_laser.txt') as f:
    txt=f.read()
    lines=txt.split('\n')
    for line in lines:
        if line: odom_laser.append( map( float, line.split(',') )  )
        
#correction          
with open('../data/data_train/correction.txt') as f:
    txt=f.read()
    lines=txt.split('\n')
    for line in lines:
        if line: correction.append( map( float, line.split(' ') )  )


#Adjust size
absolute_laser.pop(0) #delete first position 


#make np array
absolute_laser=np.array(absolute_laser)
odom_laser=np.array(odom_laser)
correction=np.array(correction)

print('absolute laser',absolute_laser.shape)
print('odom laser',odom_laser.shape)
print('correction',correction.shape)


#stack simulated and sensor lasers and  reshape
data = np.hstack((odom_laser,absolute_laser))
print("Data_train \n", data.shape)

X_train=np.reshape(data, (data.shape[0], nb_channels, nb_height, data_dim))
print('X_train',X_train.shape)

y_train=np.reshape(correction, (correction.shape[0], output_size))
print('y_train',y_train.shape)


# for i in range(data.shape[0]):
#   print(X_train[i,:])
#   print(y_train[i,:])
#   raw_input()

#INITIALIZE NETWORK
netname="CorrectionNet"
model = Sequential()
# DIMENTIONS HERE ARE INCORRECT!
# see console for the correct ones
# in: (1,362) out: 360
model.add(Convolution2D(8, 1, 3, input_shape=(nb_channels, nb_height, data_dim)))
model.add(Activation('relu'))
# # in: 360 out: 358
# model.add(Convolution2D(8, 1, 3))
# model.add(Activation('relu'))
# 358/2 -> 179
model.add(MaxPooling2D(pool_size=(1, 2), strides=(1,2)))


# in 179 out 177
model.add(Convolution2D(16, 1, 3))
model.add(Activation('relu'))
# in: 177 out: 175
model.add(Convolution2D(16, 1, 3))
model.add(Activation('relu'))
# in: 180/2 -> 90
model.add(MaxPooling2D(pool_size=(1, 2), strides=(1,2)))

# in 90
model.add(Convolution2D(16, 1, 3))
model.add(Activation('relu'))
# in: 90 out: 90
model.add(Convolution2D(16, 1, 3))
model.add(Activation('relu'))
# in: 90/2 -> 45
model.add(MaxPooling2D(pool_size=(1, 2), strides=(1,2)))
model.summary()
# In 45*16 = 720
model.add(Flatten())
model.add(Dense(512))
model.add(Activation('relu'))
model.add(Dense(256))
model.add(Activation('relu'))
model.add(Dense(3))

#model.load_weights('weights.h5')
optim=Adam(lr=0.0001, beta_1=0.9, beta_2=0.999, epsilon=1e-08)
model.compile(loss='mse', optimizer=optim)

#run training
model.fit(X_train, y_train, nb_epoch=epoch,batch_size=batch_size,callbacks=[ModelCheckpoint('weights.h5')])
# save as JSON model
json_string = model.to_json()
open(netname+'.json', 'w').write(json_string)
#save the weights
model.save_weights(netname+'.h5')




