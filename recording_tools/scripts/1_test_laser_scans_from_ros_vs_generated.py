import numpy as np
#from Laser import Laser, Map
from laser import Laser, Map
import math
import matplotlib.pyplot as plt
from copy import copy

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

# Read positions
positions = np.loadtxt('../data_pose.txt')

# Read corresponding laser scans
#laser_scans = np.loadtxt('../map_scans.txt') #, delimiter=','

# Go through each position and generate a new scan
# Validate each generated scan against recorded ones
counter = 0
all_ranges = []
n = 2000
for i in range(n):
  ranges = laser.scan(positions[i,0], positions[i,1], positions[i,2])
  all_ranges.append(copy(ranges))

  if not counter%1000: print('iter', counter)
  counter+=1

all_ranges=np.array(all_ranges)
np.savetxt('../map_scans_py.txt', all_ranges)
#print("MSE", np.sum(np.abs(all_ranges-laser_scans))/all_ranges.shape[0])
