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

ant_colony = AntColony(motion_consumtion, num_ants=80, num_iterations=50, 
                       evaporation_rate=0.5, alpha=1, beta=1)

aco_path, aco_cost = ant_colony.find_shortest_path()

print(f'This is taking power consumtion as cost measurement for ACO, \n\
      where speed is changing according to wind impact. \n\
      ACO uses angles where wind accelerates UAV and avoids high \n\
      power consumtion paths, hence better results. \n\
      Much more realistic representation of power consumtion')

print(f'Total milliamphour consumtion: {aco_cost}')

print(f'Path sequence: {aco_path}')

plot(centroids=centroids, terrain=terrain, motion=motion, path=aco_path)

"""
   Finish up with Cluster_Path and finally add Hovering Power consumtion to
   total power consumtion equation. I already have power consumtion for hovering
   need to figgure out time required for hovering and power from WPT, which better
   to take as a static formula (with fixed Energy (Joules) required for each sensor)
"""

