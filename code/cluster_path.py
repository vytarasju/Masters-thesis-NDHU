import numpy as np
from sklearn.cluster import KMeans
import math
import statistics

"""
   Define XMeans for XYZ specifically (since it already gives precise terrain distances)
   and create clusters, so that each cluster would fit in a specific radius
   to ensure, that at each SN centroid WPT would reach all of the sensors
"""
def clusterXMeans():
    return

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

# Takes distance between each point and converts it to motion of a drone
# Was made originally, because dirrect path between points was going through terrain
def getMotion(centroids, terrain, num_points=20, elevation=0.1, 
              cost = 'distance', speed = 0, UAV_parameters = 0, wind = 0):
    iteration = 0
    motion_matrix = []
    cost_matrix = []

    # Error handling for wrong parameter inputs
    if cost == 'consumption' and (speed == 0 or UAV_parameters == 0):
        print('ERROR: consumption requires speed and UAV_parameters for getMotion parameters')
        return
    
    if cost != 'consumption' and cost != 'distance':
        print('ERROR: no such cost parameter is defined')
        return

    for index1, point1 in enumerate(centroids):
        cost_matrix.append([])
        for index2, point2 in enumerate(centroids):
            if np.array_equal(point1, point2): 
                cost_matrix[index1].append(0.0)
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
                
                if cost == 'consumption':
                    
                    wind_speed = wind[terrain_atpoint_index][3]
                    wind_angle = wind[terrain_atpoint_index][3]

                    # Calculate the angle with the reverse of y-axis 
                    # (since windninja clasifies 0 as negative y-axis)
                    # So if the angle matches with the wind, then wind is behind drone - tailwind
                    angle_radians = math.atan2(x_difference, y_difference)
                    angle_degrees = math.degrees(angle_radians)
                    # Make sure angle is between 0 and 360
                    angle_degrees = (angle_degrees + 360) % 360
                    # Get angle reverse of y-axis
                    UAV_angle = (angle_degrees + 180) % 360

                    """
                       get speed of wind against UAV when it is on angle
                       then provide that speed to propulsion power equation
                       and then calculate the time required for that distance (each ith segment)
                       and then output mAh battery consumption (each ith segment)
                       this will give a matrix with battery consumption with respect to directional wind
                    """


            if cost == 'distance': cost_matrix[index1].append(distance)
            if cost == 'consumption': cost_matrix[index1].append(UAV_angle)
            iteration += 1
    return motion_matrix, np.array(cost_matrix)

        
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

# Takes distance and 
def getMotionConsumtion(centroids, terrain, num_points=20, elevation=0.1):
    return