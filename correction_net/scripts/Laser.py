import math, copy, sys
import numpy as np

class Map(object):
  def __init__(self, size_x, size_y, offset_x, offset_y, resolution, data):
    self.size_x_ = size_x
    self.size_y_ = size_y
    self.offset_x_ = offset_x
    self.offset_y_ = offset_y
    self.resolution_ = resolution
    self.data_ = data

  def world2map(self, x, y):
    map_x = np.round( (x-self.offset_x_)/self.resolution_ )
    map_y = np.round( (y-self.offset_y_)/self.resolution_ )
    if map_x<0 or map_y<0 or map_x>=self.size_x_ or map_y>=self.size_y_: return 0, 0, False
    return map_x, map_y, True

  def map2world(self, map_x, map_y):
    x = (map_x + 0.5)*self.resolution_ + self.offset_x_
    y = (map_y + 0.5)*self.resolution_ + self.offset_y_
    return x, y

class Laser(object):
  def __init__(self, max_range, min_angle, resolution_angle, no_of_beams, noise_variance, map_ptr):
    self.max_range_ = max_range
    self.min_angle_ = min_angle
    self.resolution_angle_ = resolution_angle
    self.no_of_beams_ = no_of_beams
    self.noise_variance_ = noise_variance
    self.map_ptr_ = map_ptr
    self.ranges_ = np.zeros(no_of_beams)

  def scan(self, x, y, theta):
    current_angle = self.updateAngle(theta, self.min_angle_)
    for r in range(self.ranges_.shape[0]):
      self.ranges_[r] = self.rayCast(x, y, current_angle)
      current_angle = self.updateAngle(current_angle, self.resolution_angle_)
    return self.ranges_

  def rayCast(self, x, y, theta):
    start_x, start_y, is_ok = self.map_ptr_.world2map(x,y)
    if not is_ok: return 0

    # Initialization
    direction = np.zeros(2, dtype="float64")
    direction[0] = np.cos(theta)
    direction[1] = np.sin(theta)

    origin = np.zeros(2, dtype="float64")
    origin[0] = x
    origin[1] = y

    current = np.zeros(2, dtype="float64")
    current[0] = start_x
    current[1] = start_y

    voxelBorder = np.zeros(2, dtype="float64")
    voxelBorder[0], voxelBorder[1] = self.map_ptr_.map2world(current[0], current[1])
    voxelBorder[0] -= 0.5 * self.map_ptr_.resolution_
    voxelBorder[1] -= 0.5 * self.map_ptr_.resolution_

    step = np.zeros(2, dtype="float64")
    tMax = np.zeros(2, dtype="float64")
    tDelta = np.zeros(2, dtype="float64")

    # Compute step direction
    for i in range(2):
      if direction[i]>0.0: step[i] = 1
      elif direction[i]<0.0: step[i] = -1
      else: step[i] = 0

      if step[i]!=0:
        if step[i]==1:
          voxelBorder[i] += float( step[i] * self.map_ptr_.resolution_ * 1.0 )
        tMax[i] = (voxelBorder[i] - origin[i]) / direction[i]
        tDelta[i] = self.map_ptr_.resolution_ / abs(direction[i])
      else:
        tMax[i] = sys.float_info.max
        tDelta[i] = sys.float_info.max

    # Incremental phase
    while True:

      # Find minimum tMax
      if tMax[0] < tMax[1]: dim = 0
      else: dim = 1

      # Advance in direction of dim
      current[dim] += step[dim]
      tMax[dim] += tDelta[dim]

      if np.sqrt((current[0] - start_x)*(current[0] - start_x) + (current[1] - start_y)*(current[1] - start_y)) * self.map_ptr_.resolution_ > self.max_range_:
        return self.max_range_
      else:
        if current[0] < 0 or current[1] < 0 or current[0]>=self.map_ptr_.size_x_ or current[1]>=self.map_ptr_.size_y_:
          flag = False
        else:
          value = self.map_ptr_.data_[current[0], current[1]]
          flag = True
        if flag:
          if value < 0.55:
            mu = 0
            sigma = np.sqrt(self.noise_variance_)
            noise = sigma * np.random.randn() + mu
            return np.sqrt((current[0] - start_x)*(current[0] - start_x) + (current[1] - start_y)*(current[1] - start_y)) * self.map_ptr_.resolution_ + noise
        else:
          return self.max_range_

  def updateAngle(self, angle, increment):
    angle += increment
    if angle > math.pi*2:
      angle -= math.pi*2
    return angle