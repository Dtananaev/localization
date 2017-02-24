from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from Laser import Laser, Map
import math

if __name__ == '__main__':
  # Load map
  # im_map = Image.open("map.png").convert('L')
  # (width, height) = im_map.size
  # occ_map = np.array(list(im_map.getdata()), dtype="float64")
  # for i in range(occ_map.shape[0]):
  #   if occ_map[i] == 205.0: occ_map[i] = 1
  #   else: occ_map[i] = (occ_map[i]/255)
  # occ_map.resize(height, width)
  #occ_map.resize(width, height)

  # map.txt is recorded fomr ROS msg
  width = 617
  height = 467
  occ_map = np.loadtxt('map.txt', delimiter=",", dtype="float64")
  for i in range(occ_map.shape[0]):
    if occ_map[i] >= 0:
      occ_map[i] = 1.0-(occ_map[i]/100.0)
    else:
      occ_map[i] = 1
  occ_map.resize(height, width)
  occ_map = occ_map.T

  plt.imshow(occ_map, cmap='Greys_r')
  plt.show()

  offset_x =0.0
  offset_y =0.0
  resolution = 0.1

  max_range = 50.0
  no_of_beams = 181
  min_angle = -math.pi/2.0
  resolution_angle = math.pi/(no_of_beams-1)
  noise_variance = 0.0

  map_obj = Map(height, width, offset_x, offset_y, resolution, occ_map)
  laser = Laser(max_range, min_angle, resolution_angle, no_of_beams, noise_variance, map_obj)

  robot_pos_x = 6.0
  robot_pos_y = 5.0
  robot_theta = 0.0 * (math.pi/180.0)

  laser_pos_x = 1.2
  laser_pos_y = 0.0
  laser_angle = 0.0 * (math.pi/180.0)

  sin_theta = math.sin(robot_theta)
  cos_theta = math.cos(robot_theta)
  x = robot_pos_x + laser_pos_x * cos_theta - laser_pos_y*sin_theta
  y = robot_pos_y + laser_pos_x * sin_theta + laser_pos_y*cos_theta
  theta = robot_theta + laser_angle

  if theta > math.pi:
    theta -= math.pi*2.0
  if theta < -math.pi:
    theta += math.pi*2.0


  ranges = laser.scan(x,y,theta)

  print(ranges)
  plt.plot(ranges)
  plt.show()

  expexted = np.array([50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 25.71575355529785, 19.6254940032959, 14.434680938720703, 14.44991397857666, 14.477914810180664, 14.5, 14.340153694152832, 13.152946472167969, 13.184840202331543, 13.238202095031738, 13.277424812316895, 13.34166431427002, 13.388054847717285, 13.437261581420898, 13.516286849975586, 13.572399139404297, 13.661624908447266, 13.724431037902832, 13.823531150817871, 13.892804145812988, 14.001428604125977, 14.115594863891602, 14.194717407226562, 14.317821502685547, 14.446106910705566, 14.579438209533691, 14.717677116394043, 14.860686302185059, 15.008331298828125, 15.16047477722168, 15.316984176635742, 15.477725982666016, 15.642570495605469, 15.868522644042969, 16.042444229125977, 16.220048904418945, 16.46238136291504, 16.71077537536621, 16.964963912963867, 17.22469139099121, 17.48971176147461, 17.75978660583496, 18.03468894958496, 18.384777069091797, 18.669761657714844, 21.156795501708984, 20.8197021484375, 20.48926544189453, 20.229928970336914, 19.912307739257812, 19.663671493530273, 19.360010147094727, 19.123023986816406, 18.89153289794922, 18.66574478149414, 18.44586753845215, 18.232114791870117, 18.075950622558594, 17.873443603515625, 17.677669525146484, 17.53539276123047, 17.351945877075195, 17.219175338745117, 17.090641021728516, 16.96643829345703, 16.807737350463867, 16.694011688232422, 16.584932327270508, 16.480594635009766, 16.381086349487305, 16.286497116088867, 16.19691276550293, 16.112417221069336, 16.033090591430664, 15.959010124206543, 16.834487915039062, 18.201099395751953, 19.866806030273438, 42.23564910888672, 42.10475158691406, 42.00238037109375, 41.91145324707031, 41.82164764404297, 41.75547790527344, 41.700958251953125, 41.65297317504883, 41.62355041503906, 41.60588836669922, 41.600120544433594])
  print np.array(expexted)
  plt.plot(expexted)
  plt.show()

  for i in range(expexted.shape[0]):
    #print expexted[i] - ranges[i]
    assert( abs(expexted[i]-ranges[i])<0.00009 )

  print('All ok!')