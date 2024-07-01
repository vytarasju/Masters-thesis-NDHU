from read_write_data import *
from cluster_path import convertXYZtoMeters, clusterXMeansChargeTime
from terrain_sensors_generation import generateSensorsDensity
from measurement import IoTDevice, UAV, WPT

"""Init Parameters Begin"""
sensors_num = 50

terrain_resolution = 30
density = 5
starting_points = 6

min_hover_WPT = 2 #meters
angle_WPT = 120
"""Init Parameters End"""

"""Terrain/Wind/Sensor Begin"""
terrain = readTerrainXYZ('output_NASADEM_old.xyz')
terrain = convertXYZtoMeters(terrain)

wind = readWindASCWindninja(terrain, "output_NASADEM_20_3_30m_vel.asc", "output_NASADEM_20_3_30m_ang.asc")
sensors = generateSensorsDensity(terrain, sensors_num, density, starting_points, terrain_resolution)
"""Terrain/Wind/Sensor End"""

"""Writting Begin"""
writeTerrain(terrain)
writeSensors(sensors)
writeWind(wind)
"""Writting End"""