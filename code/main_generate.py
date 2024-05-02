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
cluster_num = 5
sensors_num = 200
"""END Parameter Defintion"""

# generateTerrain - randomized terrain data
# terrain = generateTerrain(width, length, scale, octaves, persistence, lacunarity, seed)

# local_terrain - real terrain data, 
terrain = local_terrain = readTerrainXYZWindninja('local_small.xyz')

# Read wind data from ASC and make a list with terrain data together
wind = readWindASCWindninja(terrain, "local_small_20_3_109m_vel.asc", "local_small_20_3_109m_ang.asc")

sensors = generateSensors(terrain, sensors_num)
centroids = clusterKMeans( terrain, sensors, cluster_num)

writeTerrain(terrain)
writeSensors(sensors)
writeCentroids(centroids)
writeWind(wind)
