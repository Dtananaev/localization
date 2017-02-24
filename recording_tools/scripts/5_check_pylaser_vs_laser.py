import numpy as np
from Laser import Laser, Map
import math

def MSE(vec1, vec2):
  assert(vec1.shape[0] == vec2.shape[0])
  return np.sum(np.square(vec1-vec2))/vec1.shape[0]

height = 467
width = 617
offset_x =0.0
offset_y =0.0
resolution = 0.1
# Create map and laser scans
occ_map = Map.readFromTXT('../map.txt', width, height, offset_x, offset_y, resolution)

max_range = 50.0
no_of_beams = 181
min_angle = -math.pi/2.0
resolution_angle = math.pi/(no_of_beams-1)
noise_variance = 0.0
laser_pos_x = 1.2
laser_pos_y = 0.0
laser_angle = 0.0 * (math.pi/180.0)
laser = Laser(max_range, min_angle, resolution_angle, no_of_beams, noise_variance, occ_map, laser_pos_x, laser_pos_y, laser_angle)


# load positions
positions = np.loadtxt('../data_pose.txt')
# load laser
ranges = np.loadtxt('../map_scans.txt')

assert( positions.shape[0] == ranges.shape[0] )

n = 5
# sample n random scans from
pidx = np.random.randint(0, positions.shape[0], n)

sum_mse = 0
clump = 11
# Generate scans with python and compare
for i in range(pidx.shape[0]):
  pyranges = laser.scan(positions[pidx[i],0], positions[pidx[i],1], positions[pidx[i],2])
  sum_mse +=  MSE(np.around(pyranges,clump), np.around(ranges[pidx[i]],clump))
print('MSE ', sum_mse/n)
