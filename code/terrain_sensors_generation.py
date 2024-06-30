import numpy as np
from noise import pnoise2
import random
import math

"""Example of Parameter Selection
length = width = 100
scale = 100
octaves = 6
persistence = 0.5
lacunarity = 1.8
seed = 15
"""

# Generate height values with perlin noise
def generateTerrain(width, length, scale, octaves, persistence, lacunarity, seed):
    terrain_z = np.zeros((width, length))
    for i in range(width):
        for j in range(length):
            terrain_z[i][j] = pnoise2(i / scale,
                                     j / scale,
                                     octaves=octaves,
                                     persistence=persistence,
                                     lacunarity=lacunarity,
                                     repeatx=1024,
                                     repeaty=1024,
                                     base=seed)
            
    # Create gradual x and y for terrain
    terrain_x = np.linspace(0, 1, width)
    terrain_y = np.linspace(0, 1, length)
    terrain_x, terrain_y = np.meshgrid(terrain_x, terrain_y)

    #store terrain x, y, z into one list
    terrain = []
    for i in range(length):
            for j in range(width):
                terrain.append([terrain_x[i][j], terrain_y[i][j], terrain_z[i][j]])
    return np.array(terrain)

# Takes random terrain points to generate sensor locations
def generateSensors(terrain, sensors_num):
    sensors = []
    terrain_len = len(terrain) - 1
    # Take random index member in terrain for a random sensor placement
    for sensor in range(sensors_num):
        index = random.randint(1, terrain_len)
        sensors.append(terrain[index])
    return np.array(sensors)

# Creates random starting points which then acts as center points for other sensors to be generated
# Starting points are generated far enough from terrain edge, so that generated sensors would not be out of terrain bounds
def generateSensorsDensity(terrain, sensors_num, density, starting_points, terrain_resolution):
    sensors = []
    sensors_num -= starting_points
    sensors_per_center = math.floor(sensors_num / starting_points)
    X_min_center = min([point[0] for point in terrain]) + density
    X_max_center = max([point[0] for point in terrain]) - density
    Y_min_center = min([point[1] for point in terrain]) + density
    Y_max_center = max([point[1] for point in terrain]) - density

    # Take random index member in terrain for a random sensor placement
    for center in range(starting_points):
        # If sensors_per_center is a float, then last center will have the remaining sensors
        if center == (starting_points - 1): sensors_per_center = sensors_num - ((starting_points - 1) * sensors_per_center)
        X_center = random.uniform(X_min_center, X_max_center)
        Y_center = random.uniform(Y_min_center, Y_max_center)
        terrain_atpoint_index = np.argmin((terrain[:, 0] - X_center)**2 + (terrain[:, 1] - Y_center)**2)
        Z_center = terrain[terrain_atpoint_index][2]
        center = [X_center, Y_center, Z_center]
        sensors.append(np.array(center))

        for sensor in range(sensors_per_center):
            X_sensor_offset = np.random.uniform(-density, density)
            Y_sensor_offset = np.random.uniform(-density, density)
            X_sensor = center[0] + X_sensor_offset
            Y_sensor = center[1] + Y_sensor_offset
            terrain_atpoint_index = np.argmin((terrain[:, 0] - X_sensor)**2 + (terrain[:, 1] - Y_sensor)**2)
            Z_sensor = terrain[terrain_atpoint_index][2]
            sensor = [X_sensor, Y_sensor, Z_sensor]
            distance = round(np.sqrt(np.sum(np.power((sensor - terrain[terrain_atpoint_index]), 2))), 2)
            
            if (distance <= terrain_resolution):
                sensors.append(np.array(sensor))
    return np.array(sensors)