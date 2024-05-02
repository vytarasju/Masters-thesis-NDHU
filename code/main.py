from cluster_path import *
from read_write_data import *
from plot import *
from algorithms import *
from measurement import *

terrain = readCSV('terrain_data.csv')
sensors = readCSV('sensor_coordinates.csv')
centroids = readCSV('centroid_coordinates.csv')
wind = readCSV('wind_data.csv')

UAV_speed = 20 #meters/second
UAV_elevation = 200 #meters
UAV_steps = 40 #steps between points

motion, motion_distance = getMotionAndDistance(centroids, terrain, num_points=UAV_steps, 
                                               elevation=UAV_elevation)

ant_colony = AntColony(motion_distance, num_ants=80, num_iterations=50, 
                       evaporation_rate=0.5, alpha=1, beta=1)
aco_path, aco_distance = ant_colony.find_shortest_path()

measurements = DefineMeasurements(height_width=100, terrain=terrain, 
                                  distance=aco_distance, motion_distance=motion_distance)

power_consumtion_moving = measurements.getPropulsionPowerConsumtion(UAV_speed)
power_consumtion_hover = measurements.getPropulsionPowerConsumtion(0)

distance_measured, motion_measured = measurements.getMeasurements(type='watts', speed=UAV_speed)

plot(centroids=centroids, terrain=terrain, motion=motion, path=aco_path)

