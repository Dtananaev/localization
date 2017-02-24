import numpy as np
#from Laser import Laser, Map
from laser import Laser, Map
import math
from copy import copy
import time

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

positions = np.loadtxt('odometry_gen.txt')

counter =0
all_ranges = []
for i in range(positions.shape[0]):
  #start = time.clock()
  ranges = laser.scan(positions[i,0], positions[i,1], positions[i,2])
  #print('Time', time.clock() - start)
  all_ranges.append(copy(ranges))
  if not counter%1000: print("iter ", counter)
  counter += 1

np.savetxt('../map_scans_odom.txt', np.array(all_ranges))