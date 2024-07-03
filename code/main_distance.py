from cluster_path import *
from read_write_data import *
from plot import *
from algorithms import *
from measurement import *
from terrain_sensors_generation import *

terrain = readTerrainXYZ('output_NASADEM_old.xyz')
terrain = convertXYZtoMeters(terrain)
wind = readWindASCWindninja(terrain, "output_NASADEM_20_3_30m_vel.asc", "output_NASADEM_20_3_30m_ang.asc")


"""BEGIN Motion and WPT Parameter Defintion"""
# For X-Means
min_hover_WPT = 2 #meters
angle_WPT = 120 #degrees

# For UAV
UAV_speed = 30 #meters/second
UAV_elevation = 39 #meters
UAV_steps = 20 #steps between points
"""END Parameter Defintion"""

"""BEGIN Sensor and Terrain Parameter Definition"""
terrain_resolution = 30 #according terrain data used

sensors_num = 500
density = 1.5
starting_points = 3

# For iteration process
"""END Sensor and Terrain Parameter Definition"""

drone = UAV()
iot = IoTDevice()
wpt = WPT()

provide_charge = iot.batteryConsumtionGivenTime(0, drone.minimum_operation_time)

sensors = generateSensorsDensity(terrain, sensors_num, density, starting_points, terrain_resolution)
centroids, wpt_area, cluster_charge_time, K_value = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge)

movement_matrix, time_matrix, motion_matrix = getMotion(centroids, terrain, num_points=UAV_steps, elevation=UAV_elevation)

print(motion_matrix)

ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)

aco_path, aco_distance = ant_colony.find_shortest_path()

distance_converted, motion_converted = drone.convertDistancetoMeasurements(aco_distance, movement_matrix, type='milliamphours')

print(f'This is taking just distance as cost measurement for AC')

print(f'Total milliamphour consumtion: {distance_converted}')

print(f'Path sequence: {aco_path}')

plot(centroids=centroids, terrain=terrain, path=aco_path, motion=motion_matrix)




