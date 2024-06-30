import numpy as np
import csv
import os

path_csv = "/home/vytska/thesis/code/csv"
path_windninja = "/home/vytska/thesis/code/windninja/"
os.chdir(path_csv)

#reading terrain data
#return terrain list
def readCSV(filename):
    # Read Terrain Data
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        header = next(reader)  # Skip header row
        rows = list(reader)

    # Convert data to numpy arrays
    data = np.array(rows, dtype=float)
    return data

# Save terrain data to CSV
def writeTerrain(terrain):
    with open('terrain_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['terrain_x', 'terrain_y', 'terrain_z'])  # Write header
        for point in terrain:
                writer.writerow([point[0], point[1], point[2]])
    print("Terrain data saved to terrain_data.csv")

# Save sensor data to CSV
def writeSensors(sensors):
    with open('sensor_coordinates.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['sensor_x', 'sensor_y', 'sensor_z'])  # Write header
        for sensor in sensors:
                writer.writerow([sensor[0], sensor[1], sensor[2]])
    print("Sensors coordinates saved to sensor_coordinates.csv")

def writeCentroids(centroids):
    with open('centroid_coordinates.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['centroid_x', 'centroid_y', 'centroid_z'])  # Write header
        for centroid in centroids:
                writer.writerow([centroid[0], centroid[1], centroid[2]])
    print("Centroids coordinates saved to centroid_coordinates.csv")

def readTerrainXYZWindninja(filename):
    data = []
    with open(path_windninja + "converted/" + filename, 'r') as file:
        for line in file:
            parts = line.split()
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            data.append([x, y, z])
    return np.array(data)

def readWindASCWindninja(terrain, filename_vel, filename_ang):
    def readASCFile(filename, data_type):
        data_list = []
        with open(path_windninja + "wind/" + filename, 'r') as f:
            # Skip the first 6 lines
            for _ in range(6):
                next(f)        
            for line in f:
                data_list.extend(map(data_type, line.strip().split()))
        return data_list

    # Lists for velocity and angle of wind
    data_vel = readASCFile(filename_vel, float)
    data_ang = readASCFile(filename_ang, int)

    wind = []
    for index, point in enumerate(terrain):
         wind.append([point, data_vel[index], data_ang[index]])

    return wind

def writeWind(wind):
    with open('wind_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['terrain_x', 'terrain_y', 'terrain_z', 'wind_velocity', 'wind_angle'])  # Write header
        for point in wind:
                writer.writerow([point[0][0], point[0][1], point[0][2], point[1], point[2]])
    print("Wind data saved to wind_data.csv")

def writeWPT(charge_time, charge_consumption, charging_points):
    with open('WPT_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['WPT_charge_time', charge_time])
        writer.writerow(['WPT_charge_consumption', charge_consumption])
        writer.writerow(['center', 'cluster_hover_height', 'furthest_sensor_center_distance'])
        for point in charging_points:
                writer.writerow(point)
    print("WPT data saved to WPT_data.csv")

def readWPT(filename):
    data = []
    with open(filename, mode='r') as file:
        reader = list(csv.reader(file))
        for index, line in enumerate(reader):
            if index > 2:
                center = np.array(line[0].strip('[]').split(), dtype=float)
                data.append([center, float(line[1]), float(line[2])])
            elif index == 0: WPT_charge_consumption = line[1]
            elif index == 1: WPT_charge_time = line[1]
    return WPT_charge_time, WPT_charge_consumption, data