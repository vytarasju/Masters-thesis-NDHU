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

drone = UAV()

movement_matrix, time_matrix, motion_matrix = getMotion(centroids, terrain, num_points=UAV_steps, elevation=UAV_elevation)

ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)

aco_path, aco_distance = ant_colony.find_shortest_path()

distance_converted, motion_converted = drone.convertDistancetoMeasurements(aco_distance, movement_matrix, type='milliamphours')

print(f'This is taking just distance as cost measurement for AC')

print(f'Total milliamphour consumtion: {distance_converted}')

print(f'Path sequence: {aco_path}')

plot(centroids=centroids, terrain=terrain, path=aco_path, motion=motion_matrix)




