import numpy as np
from sklearn.cluster import KMeans
import math
import statistics

def clusterKMeans(terrain, sensors, n_clusters):

    def getCenterPoint(terrain):
        x, y = statistics.median(terrain[:, 0]), statistics.median(terrain[:, 1])
        # Takes euclidean distance between x and y values and returns index of smallest distance
        point_index = np.argmin((terrain[:, 0] - x)**2 + (terrain[:, 1] - y)**2)
        return terrain[point_index]
    
    center_point = getCenterPoint(terrain)

    #n_init set to surpress warning messages
    kmeans = KMeans(n_clusters=n_clusters, n_init=10)
    kmeans.fit(sensors)

    centroids = kmeans.cluster_centers_
    centroids = np.vstack((center_point, centroids))
    # labels = kmeans.labels_
    return centroids

#using euclidean distance between 2 points
#returning matrix of distance between each centroid points
def getDistanceCentroids(centroids):
    num_centroids = centroids.shape[0]
    distance_matrix = np.zeros((num_centroids, num_centroids))
    for i in range(num_centroids):
        for j in range(num_centroids):
            distance = round(np.sqrt(np.sum(np.power((centroids[i] - centroids[j]), 2))), 2)
            distance_matrix[i][j] = distance
            distance_matrix[j][i] = distance
    return distance_matrix

def getMotionDistance(centroids, terrain, num_points=20, elevation=0.1):
    iteration = 0
    motion_matrix = []
    distance_matrix = []

    for index1, point1 in enumerate(centroids):
        distance_matrix.append([])
        for index2, point2 in enumerate(centroids):
            if np.array_equal(point1, point2): 
                distance_matrix[index1].append(0.0)
                continue
            motion_matrix.append([[], index1, index2])
            distance = 0
            for i in range(num_points):
                t = i / (num_points - 1)  # Parameter t ranges from 0 to 1
                x = (1 - t) * point1[0] + t * point2[0]
                y = (1 - t) * point1[1] + t * point2[1]

                # Get z value from terrain
                terrain_atpoint_index = np.argmin((terrain[:, 0] - x)**2 + (terrain[:, 1] - y)**2)
                z = terrain[terrain_atpoint_index][2] + elevation
                motion_matrix[iteration][0].append((x, y, z))

                # Get distance between each segment
                if i == 0: continue
                x_difference = motion_matrix[iteration][0][i][0] - motion_matrix[iteration][0][i - 1][0]
                y_difference = motion_matrix[iteration][0][i][1] - motion_matrix[iteration][0][i - 1][1]
                z_difference = motion_matrix[iteration][0][i][2] - motion_matrix[iteration][0][i - 1][2]
                distance += math.sqrt(x_difference**2 + y_difference**2 + z_difference**2)
            distance_matrix[index1].append(distance)
            iteration += 1
    return motion_matrix, np.array(distance_matrix)

        
def convertXYZtoMeters(terrain):
    terrain_converted = []
    terrain = terrain.tolist()
    x_min, y_min = float('inf'), float('inf')

    for point in terrain:
        if point[0] < x_min: x_min = point[0]
        if point[1] < y_min: y_min = point[1]

    for point in terrain:
        point_converted = [point[0] - x_min, point[1] - y_min, point[2]]
        terrain_converted.append(point_converted)
    
    return np.array(terrain_converted)
