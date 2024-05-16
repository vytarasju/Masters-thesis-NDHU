import numpy as np
from noise import pnoise2
import random

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

def generateSensors(terrain, sensors_num):
    sensors = []
    terrain_len = len(terrain) - 1
    # Take random index member in terrain for a random sensor placement
    for sensor in range(sensors_num):
        index = random.randint(1, terrain_len)
        sensors.append(terrain[index])
    return np.array(sensors)
