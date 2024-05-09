from cluster_path import *
from read_write_data import *
from plot import *
from algorithms import *
from measurement import *

terrain = readCSV('terrain_data.csv')
sensors = readCSV('sensor_coordinates.csv')
centroids = readCSV('centroid_coordinates.csv')
wind = readCSV('wind_data.csv')

UAV_speed = 30 #meters/second
UAV_elevation = .2 #meters
UAV_steps = 40 #steps between points

measurements = DefineMeasurements(terrain=terrain)

motion, motion_consumtion = getMotion(centroids, terrain, num_points = UAV_speed,
                                      elevation=UAV_elevation, cost = 'consumption',
                                      speed = UAV_speed, UAV_parameters = measurements, wind=wind)

print(motion_consumtion.tolist())

"""
   Finish up with Cluster_Path and finally add Hovering Power consumtion to
   total power consumtion equation. I already have power consumtion for hovering
   need to figgure out time required for hovering and power from WPT, which better
   to take as a static formula (with fixed Energy (Joules) required for each sensor)
"""

"""
   I have been collecting thoughts and explanations in text files separately
   for each segment of the thesis, they will help with paper writting a lot
   since most complex theoretical analysis parts are already defined and
   roughly typed out.
"""

