from terrain_sensors_generation import *
from cluster_path import *
from read_write_data import *
from plot import *
from measurement import IoTDevice, UAV

"""BEGIN Parameter Defintion"""
# Define generated terrain parameters
length = width = 100
scale = 100
octaves = 6
persistence = 0.5
lacunarity = 1.8
seed = 15

# Number of sensors: Used by all clustering methods
sensors_num = 50
# For KMeans
cluster_num = 5 

# WPT range definition for XMeans characteristics
# Created for XYZ terrain, where it is already metered
min_hover_WPT = 2 #meters
# For XMeansChargeTime
angle_WPT = 60
# For XMeansDistance
max_distance_WPT = 500 #meters
#!!! ADD angle of WPT power delivery, so I could check if the sensor is over the angle of WPT delivery or inside it
"""END Parameter Defintion"""

# generateTerrain - randomized terrain data
# terrain = generateTerrain(width, length, scale, octaves, persistence, lacunarity, seed)

# Real terrain data
terrain  = readTerrainXYZWindninja('output_NASADEM.xyz')
# terrain  = readTerrainXYZWindninja('local_small.xyz')
# terrain = convertXYZtoMeters(terrain)

# Read wind data from ASC and make a list with terrain data together
wind = readWindASCWindninja(terrain, "output_NASADEM_20_3_30m_vel.asc", "output_NASADEM_20_3_30m_ang.asc")
# wind = readWindASCWindninja(terrain, "local_small_20_3_109m_vel.asc", "local_small_20_3_109m_ang.asc")

sensors = generateSensors(terrain, sensors_num)
# centroids = clusterKMeans(terrain, sensors, cluster_num)
# centroids = clusterXMeansDistance(terrain, sensors, min_hover_WPT, max_distance_WPT)

drone = UAV()
IOT = IoTDevice()
#drone.maximum_operation_time
provide_charge = IOT.batteryConsumtionGivenTime(0, 500)

centroids = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge)
plot(terrain=terrain, sensors=sensors, centroids=centroids)

# plot(terrain=terrain)

# writeTerrain(terrain)
# writeSensors(sensors)
# writeCentroids(centroids)
# writeWind(wind)
