from terrain_sensors_generation import *
from cluster_path import *
from read_write_data import *
from plot import *
from measurement import IoTDevice, UAV, WPT

"""BEGIN Parameter Defintion"""
# For random terrain generation
length = width = 100
scale = 100
octaves = 6
persistence = 0.5
lacunarity = 1.8
seed = 15

# Number of sensors: Used by all clustering methods
sensors_num = 50

# For sensor generation with density
terrain_resolution = 30
density = 5
starting_points = 6

# WPT range definition for XMeans characteristics
# Created for XYZ terrain, where it is already metered
min_hover_WPT = 2 #meters
# For XMeansChargeTime
angle_WPT = 120
# For XMeansDistance
max_distance_WPT = 500 #meters
"""END Parameter Defintion"""

"""Terrain/Sensors/Wind Begin"""
# generateTerrain - randomized terrain data
# terrain = generateTerrain(width, length, scale, octaves, persistence, lacunarity, seed)

# Real terrain data
terrain = readTerrainXYZWindninja('output_NASADEM_old.xyz')
terrain = convertXYZtoMeters(terrain)

# Read wind data from ASC and make a list with terrain data together
wind = readWindASCWindninja(terrain, "output_NASADEM_20_3_30m_vel.asc", "output_NASADEM_20_3_30m_ang.asc")

# sensors = generateSensors(terrain, sensors_num)
sensors = generateSensorsDensity(terrain, sensors_num, density, starting_points, terrain_resolution)
"""Terrain/Sensors/Wind Begin"""

"""Clustering Begin"""
drone = UAV()
IOT = IoTDevice()
wpt = WPT()
provide_charge = IOT.batteryConsumtionGivenTime(0, drone.maximum_operation_time)
print(f'charge up to {provide_charge} mAh')
centroids, wpt_area, charge_time = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge)
WPT_charge_consumption = wpt.chargeConsumptionGivenTime(charge_time)
"""Clustering End"""

"""Plotting Begin"""
# plot(terrain=terrain, sensors=sensors, centroids=centroids)
"""Plotting End"""

"""Writting Begin"""
writeTerrain(terrain)
writeSensors(sensors)
writeCentroids(centroids)
writeWind(wind)
writeWPT(charge_time, WPT_charge_consumption, wpt_area)
"""Writting End"""

