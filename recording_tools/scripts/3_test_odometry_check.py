import numpy as np
import math

def normalize_angle(i):
  return np.arctan2(np.sin(i), np.cos(i))

positions = np.loadtxt('../data_pose_to_gen.txt')
odometry = np.loadtxt('odometry_gen.txt')
corrections = np.loadtxt('corrections_gen.txt')

corrected = odometry + corrections
print corrected[:,2].shape
corrected[:,2] = normalize_angle(corrected[:,2])

print("MSE ", np.sum(np.abs(positions[0:-1]-corrected))/positions.shape[0])
