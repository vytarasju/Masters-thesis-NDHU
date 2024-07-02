from cluster_path import *
from read_write_data import *
from plot import *
from algorithms import *
from measurement import *

terrain = readCSV('terrain_data.csv')
sensors = readCSV('sensor_coordinates.csv')
centroids = readCSV('centroid_coordinates.csv')
wind = readCSV('wind_data.csv')

"""BEGIN Parameter Defintion"""
UAV_elevation = 39 #meters
UAV_steps = 20 #steps between points
"""END Parameter Defintion"""

drone = UAV(terrain=terrain)

movement_matrix, time_matrix, motion_matrix = getMotion(centroids, terrain, num_points=UAV_steps, elevation=UAV_elevation)

ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, 
                       evaporation_rate=0.5, alpha=1, beta=1)
aco_path, aco_distance = ant_colony.find_shortest_path()

measurements = UAV(terrain=terrain, total_cost=aco_distance, motion_cost=movement_matrix)

distance_measured, motion_measured = measurements.convertDistancetoMeasurements(type='milliamphours', speed=drone.UAV_max_speed)

print(f'This is taking just distance as cost measurement for AC')

print(f'Total milliamphour consumtion: {distance_measured}')

print(f'Path sequence: {aco_path}')

plot(centroids=centroids, terrain=terrain, path=aco_path, motion=motion_matrix)




