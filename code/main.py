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

# Set ratio to 1 for XYZ data, set to a 100 for generated data
height_width_ratio = 1

motion, motion_distance = getMotionDistance(centroids, terrain, num_points=UAV_steps, 
                                               elevation=UAV_elevation)

ant_colony = AntColony(motion_distance, num_ants=80, num_iterations=50, 
                       evaporation_rate=0.5, alpha=1, beta=1)
aco_path, aco_distance = ant_colony.find_shortest_path()

measurements = DefineMeasurements(height_width=height_width_ratio, terrain=terrain, 
                                  total_distance=aco_distance, motion_distance=motion_distance)

power_consumtion_moving = measurements.getPropulsionPowerConsumtion(UAV_speed)
power_consumtion_hover = measurements.getPropulsionPowerConsumtion(0)

distance_measured, motion_measured = measurements.getMeasurements(type='milliamphours', speed=UAV_speed)

print(f'power: {measurements.getPropulsionPowerConsumtion(UAV_speed)}')
print(distance_measured)

# plot(centroids=centroids, terrain=terrain, motion=motion, path=aco_path)

