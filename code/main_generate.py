from terrain_sensors_generation import *
from cluster_path import *
from read_write_data import *
from plot import *

"""BEGIN Parameter Defintion"""
# Define generated terrain parameters
length = width = 100
scale = 100
octaves = 6
persistence = 0.5
lacunarity = 1.8
seed = 15

# Number of sensors
sensors_num = 50
cluster_num = 5 # Used in KMeans

# WPT range definition for XMeans characteristics
# Created for XYZ terrain, where it is already metered
min_hover_WPT = 10 #meters
max_distance_WPT = 500 #meters
"""END Parameter Defintion"""

# generateTerrain - randomized terrain data
# terrain = generateTerrain(width, length, scale, octaves, persistence, lacunarity, seed)

# local_terrain - real terrain data, 
terrain = local_terrain = readTerrainXYZWindninja('xyz-NASADEM.xyz')
terrain = convertXYZtoMeters(terrain)

# Read wind data from ASC and make a list with terrain data together
# wind = readWindASCWindninja(terrain, "local_muchsmall_0_3_11m_vel.asc", "local_muchsmall_0_3_11m_ang.asc")

# sensors = generateSensors(terrain, sensors_num)
# centroids = clusterKMeans(terrain, sensors, cluster_num)
# centroids = clusterXMeans(terrain, sensors, min_hover_WPT, max_distance_WPT)
# plot(terrain=terrain, sensors=sensors, centroids=centroids)

plot(terrain=terrain)

# writeTerrain(terrain)
# writeSensors(sensors)
# writeCentroids(centroids)
# writeWind(wind)
