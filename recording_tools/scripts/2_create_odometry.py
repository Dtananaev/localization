import numpy as np
import math
from copy import copy
import random

THRESHOLD = 0.0000000000001
np.random.seed(1337)

def MSE(vec1, vec2):
  assert(vec1.shape[0] == vec2.shape[0])
  return np.sum(np.square(vec1-vec2))/vec1.shape[0]

def normalize_angle(i):
  return math.atan2(math.sin(i), math.cos(i))

def odometry_model(pos1, pos2):
  # http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
  # slide 10
  # slide 24
  xdiff = (pos2[0]-pos1[0])
  ydiff = (pos2[1]-pos1[1])
  trans = np.sqrt( xdiff*xdiff + ydiff*ydiff )
  rot1 = math.atan2(ydiff,xdiff) - pos1[2]
  rot2 = pos2[2] - pos1[2] - rot1

  alpha1 = 0.01
  alpha2 = 0.01
  alpha3 = 0.005
  alpha4 = 0.005

  rot1_noise = random.gauss(0,alpha1*np.abs(rot1) + alpha2*trans)
  trans_noise = random.gauss(0,alpha3*trans + alpha4*(np.abs(rot1 + rot2)))
  rot2_noise = random.gauss(0,alpha1*np.abs(rot2) + alpha2*trans)

  ideal_rot1 = rot1
  ideal_rot2 = rot2
  ideal_trans = trans

  # Assume that if robot doesn't move, we just don't get any readings
  if rot1 == 0 and  rot2 == 0 and trans == 0:
    rot1_noise = 0
    rot2_noise = 0
    trans_noise = 0

  rot1  += rot1_noise
  trans += trans_noise
  rot2  += rot2_noise

  odom_x_noise = trans*np.cos(pos1[2] + rot1)
  odom_y_noise = trans*np.sin(pos1[2] + rot1)
  odom_theta_noise = rot1 + rot2


  odom_x  = pos1[0] + odom_x_noise
  odom_y  = pos1[1] + odom_y_noise
  odom_theta = normalize_angle( pos1[2] + odom_theta_noise )


  assert( abs(pos1[0] - odom_x + odom_x_noise)         < THRESHOLD )
  assert( abs(pos1[1] - odom_y + odom_y_noise)         < THRESHOLD )
  assert( abs(normalize_angle(pos1[2] - odom_theta + odom_theta_noise)) < THRESHOLD )

  new_pose = np.array([odom_x, odom_y, odom_theta])
  pose_corr = np.array([-odom_x_noise, -odom_y_noise, -odom_theta_noise])
  ideal_odom = np.array([ideal_trans, ideal_rot1, ideal_rot2])
  noisy_odom = np.array([trans, rot1, rot2])
  odom_correction = np.array([-trans_noise,-rot1_noise,-rot2_noise])
  return new_pose, pose_corr, ideal_odom, noisy_odom, odom_correction


def motion_model(old_pos, odom):
  new_pose = np.zeros(3)
  new_pose[0] = old_pos[0] + odom[0]*np.cos(old_pos[2] + odom[1])
  new_pose[1] = old_pos[1] + odom[0]*np.sin(old_pos[2] + odom[1])
  new_pose[2] = normalize_angle( old_pos[2] +  odom[1] + odom[2])
  return new_pose

# read true positions
positions = np.loadtxt('../data_pose.txt')
print positions.shape
pose_corrections = []
odom_corrections = []
total_noisy_odom = []
total_ideal_odom = []
ideal_odometry_track = [positions[0,:]]
noisy_odometry_track = [positions[0,:]]
check_odometry_corr = [positions[0,:]]
odometry_positions = [positions[0,:]]

# We will need to remove the last pose and laser scan from datasets
for i in range(positions.shape[0]-1):

  odom_pose, correction, ideal_odom, noisy_odom, odom_correction = odometry_model(positions[i,:], positions[i+1,:])

  pose_corrections.append(copy(correction))
  odom_corrections.append(copy(odom_correction))
  total_noisy_odom.append(copy(noisy_odom))
  total_ideal_odom.append(copy(ideal_odom))

  odom_pos_from_ideal = motion_model(positions[i,:], noisy_odom)
  odometry_positions.append(copy(odom_pos_from_ideal))

  ideal_odom_pos = motion_model(ideal_odometry_track[-1], ideal_odom)
  ideal_odometry_track.append(ideal_odom_pos)

  noisy_odom_pos = motion_model(noisy_odometry_track[-1], noisy_odom)
  noisy_odometry_track.append(noisy_odom_pos)

  check_odom_corr = motion_model(check_odometry_corr[-1], noisy_odom+odom_correction)
  check_odometry_corr.append(check_odom_corr)


print('MSE odom check, should be 0', MSE(np.array(ideal_odometry_track), positions))
print('MSE corrected odom check, should be 0', MSE(np.array(check_odometry_corr), positions))

# For training
np.savetxt('odometry_from_ideal.txt', np.array(odometry_positions))
np.savetxt('odom_corrections_gen.txt', np.array(odom_corrections))

# For testing 
np.savetxt('odometry_noisy.txt', np.array(total_noisy_odom))

# For debugging
np.savetxt('odometry_track_ideal.txt', np.array(ideal_odometry_track))
np.savetxt('odometry_track_noisy.txt', np.array(noisy_odometry_track))
np.savetxt('odometry_ideal.txt', np.array(total_ideal_odom))
np.savetxt('pose_corrections_gen.txt', np.array(pose_corrections))

