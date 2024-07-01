from cluster_path import *
from read_write_data import *
from plot import *
from algorithms import AntColony
from measurement import UAV

terrain = readCSV('terrain_data.csv')
sensors = readCSV('sensor_coordinates.csv')
centroids = readCSV('centroid_coordinates.csv')
wind = readCSV('wind_data.csv')
WPT_charge_time, WPT_charge_consumption, WPT_charging_areas = readWPT('WPT_data.csv')

"""BEGIN Parameter Defintion"""
UAV_speed = 30 #meters/second
UAV_elevation = 39 #meters
UAV_steps = 20 #steps between points
"""END Parameter Defintion"""

drone = UAV(terrain=terrain)

motion, consumption_matrix, time_matrix = getMotion(centroids, terrain, num_points=UAV_steps,
                                      elevation=UAV_elevation, cost='consumption',
                                      UAV_parameters=drone, wind=wind)

ant_colony = AntColony(consumption_matrix, num_ants=80, num_iterations=50, 
                       evaporation_rate=0.5, alpha=1, beta=1)

aco_path, aco_cost = ant_colony.find_shortest_path()

print(f'This is taking power consumtion as cost measurement for ACO')

print(f'Charge Consumption: Path {aco_cost} mAh, WPT {WPT_charge_consumption} mAh')

print(f'Path sequence: {aco_path}')

plot(centroids=centroids, terrain=terrain, motion=motion, path=aco_path)

