import numpy as np
from sklearn.cluster import KMeans
import math
import statistics
from measurement import UAV, WPT

# Initiate for further use
wpt = WPT()
drone = UAV()

def getCenterPoint(terrain):
    x, y = statistics.median(terrain[:, 0]), statistics.median(terrain[:, 1])
    # Takes euclidean distance between x and y values and returns index of smallest distance
    point_index = np.argmin((terrain[:, 0] - x)**2 + (terrain[:, 1] - y)**2)
    return terrain[point_index]

def clusterXMeansChargeTime(terrain, sensors, angle, lowest_hover_height, provide_charge, K_value = 'NA'):
    cluster_hover_time = []
    center_point = getCenterPoint(terrain)
    sensor_num = len(sensors)
    Check_K_ceiling, Check_K_floor = False, False
    smallest_charge_time = float('inf')
    limit_counter = 0
    
    # If K_value provided, start checking for Check_K_ceiling
    if K_value > 1: Check_K_ceiling = True
    # Check if K_value provided, if not then start K from 1 and Check_K_floor
    elif K_value == 'NA' or K_value == 1: 
        K_value = 1
        Check_K_floor = True

    # Iterate until can't increase K_value or solution is found
    while K_value <= sensor_num:
        # Find solution with current K_value
        kmeans = KMeans(n_clusters=K_value, n_init=10)
        kmeans.fit(sensors)

        # Clear list values
        wpt_area = []
        cluster_hover_time = []
        clustered_sensors = [[] for _ in range(K_value)]

        # Get arrays with each sensor assigned to clusters
        for index, label in enumerate(kmeans.labels_):
            clustered_sensors[label].append(sensors[index])
        sensor_center_distance = [[] for _ in range(K_value)]

        # Get distance for each sensor to its cluster center
        for index, center in enumerate(kmeans.cluster_centers_):
            for sensor in clustered_sensors[index]:
                distance = round(np.sqrt(np.sum(np.power((center - sensor), 2))), 2)
                sensor_center_distance[index].append(distance)
        total_charge_time = 0

        for index, center in enumerate(kmeans.cluster_centers_):
            furthest_sensor_center_distance = max(sensor_center_distance[index])
            # Get XYZ of the furthest sensor from center of cluster and get straight line 
            # distance to the center point XY values at the same Z level as the furthest point
            furthest_sensor_center = clustered_sensors[index][sensor_center_distance[index].index(furthest_sensor_center_distance)]
            furthest_sensor_at_centerline = [center[0], center[1], furthest_sensor_center[2]]
            diameter = round(np.sqrt(np.sum(np.power((furthest_sensor_center - furthest_sensor_at_centerline), 2))), 2)
            # Get lowest height for UAV to reach all sensors
            cluster_hover_height = diameter / math.tan(math.radians(angle / 2))
            # Check if hover point is not lower than set boundary
            if cluster_hover_height < lowest_hover_height: cluster_hover_height = lowest_hover_height
            # Get exact hovering point of UAV
            cluster_hover_point = [center[0], center[1], center[2] + cluster_hover_height]
            furthest_sensor_UAV_distance = 0

            for sensor in clustered_sensors[index]:
                # Get distance between hover point and furthest sensor
                distance = round(np.sqrt(np.sum(np.power((cluster_hover_point - sensor), 2))), 2)
                if distance > furthest_sensor_UAV_distance: furthest_sensor_UAV_distance = distance
            
            charge_time = wpt.chargeTime(furthest_sensor_UAV_distance, provide_charge)
            total_charge_time += charge_time
            cluster_hover_time.append(charge_time)
            wpt_area.append([center, cluster_hover_height, furthest_sensor_center_distance])

        # If Charging time is equals to or exceedes UAV operation time, this k is not the solution
        # print(f'k: {K_value}, total_charge: {total_charge_time / 60} min')
        if total_charge_time >= drone.minimum_operation_time:
            print(f'Cluster NOTSOLUTION: charge time {(total_charge_time/60):.2f} min at {K_value}K')
            if Check_K_ceiling == True: 
                limit_counter += 1
                print(f'K Ceiling {limit_counter}')
                if limit_counter >= 3: 
                    print('ITERATION END: K Value Ceiling reached')
                    return 0
            elif Check_K_floor == True:
                # If charge time is constantly increasing, then K_floor has been reached
                if total_charge_time > smallest_charge_time: limit_counter += 1
                else: 
                    smallest_charge_time = total_charge_time
                    limit_counter = 0
                print(f'K Floor {limit_counter}: {(total_charge_time/60):.2f}, {(smallest_charge_time/60):.2f}')
                if limit_counter >= 20:
                    print('ITERATION END: K Value Floor reached')
                    return 0
            K_value += 1
            continue
        else:
            centroids = np.vstack((center_point, kmeans.cluster_centers_))
            return centroids, wpt_area, cluster_hover_time, K_value
        
    # If solution was not found, no solution exists
    print('ITERATION END: K Value reached sensor_num value')
    return 0

#using euclidean distance between 2 points
#returning matrix of distance between each centroid points
def getDistanceCentroids(centroids):#
    num_centroids = centroids.shape[0]
    distance_matrix = np.zeros((num_centroids, num_centroids))
    for i in range(num_centroids):
        for j in range(num_centroids):
            distance = round(np.sqrt(np.sum(np.power((centroids[i] - centroids[j]), 2))), 2)
            distance_matrix[i][j] = distance
            distance_matrix[j][i] = distance
    return distance_matrix

# Helper funciton for getMotion to calculate power needed for UAV flying in wind
def getPowerInWind(wind_speed, wind_angle = 0, speed = 0, x_difference = 0, y_difference = 0, movement = 'fly'):
    UAV_speed_in_wind = 0

    # Calculate the angle with the reverse of y-axis 
    # (since windninja clasifies 0 as negative y-axis)
    angle_radians = math.atan2(x_difference, y_difference)
    angle_degrees = math.degrees(angle_radians)
    # Make sure angle is between 0 and 360
    angle_degrees = (angle_degrees + 360) % 360
    # Get angle reverse of y-axis
    UAV_angle = (angle_degrees + 180) % 360

    # Get angle of wind from UAV perspective
    Wind_to_UAV_angle = 360 - (((UAV_angle + 180) % 360) - wind_angle)
    
    # Get actual UAV speed with effect of wind
    # between 90 and 270 from UAV perspective - tailwind = increases speed
    # between 0, 90 and 270, 360 wind is against UAV - slowing it down
    if movement == 'fly': UAV_speed_in_wind = speed - (wind_speed * math.cos(math.radians(Wind_to_UAV_angle)))
    elif movement == 'hover': UAV_speed_in_wind = wind_speed
    return drone.getPropulsionPowerConsumtion(UAV_speed_in_wind), UAV_speed_in_wind


# Takes distance between each point and converts it to motion of a drone
# Was made originally, because dirrect path between points was going through terrain
def getMotion(centroids, terrain, num_points=20, elevation=0.1, cost = 'distance', wind = 0):
    iteration = 0
    motion_matrix = []
    movement_matrix = []
    time_matrix = []
    speed = drone.UAV_max_speed
    
    # Error handling for wrong parameter inputs
    if cost != 'consumption' and cost != 'distance':
        print('ERROR: no such cost parameter is defined')
        return

    for index1, point1 in enumerate(centroids):
        movement_matrix.append([])
        time_matrix.append([])
        for index2, point2 in enumerate(centroids):
            if np.array_equal(point1, point2):
                movement_matrix[index1].append(0.0)
                time_matrix[index1].append(0.0)
                continue
            motion_matrix.append([[], index1, index2])
            total_distance, total_milliamphour, total_time = 0, 0, 0

            # Begin path segmentation
            for i in range(num_points):
                t = i / (num_points - 1)  # Parameter t ranges from 0 to 1
                x = (1 - t) * point1[0] + t * point2[0]
                y = (1 - t) * point1[1] + t * point2[1]

                # Get z value from terrain
                terrain_atpoint_index = np.argmin((terrain[:, 0] - x)**2 + (terrain[:, 1] - y)**2)
                z = terrain[terrain_atpoint_index][2] + elevation
                motion_matrix[iteration][0].append([x, y, z])

                # Get distance between each segment
                if i == 0: continue
                x_difference = motion_matrix[iteration][0][i][0] - motion_matrix[iteration][0][i - 1][0]
                y_difference = motion_matrix[iteration][0][i][1] - motion_matrix[iteration][0][i - 1][1]
                z_difference = motion_matrix[iteration][0][i][2] - motion_matrix[iteration][0][i - 1][2]
                distance = math.sqrt(x_difference**2 + y_difference**2 + z_difference**2)
                
                if cost == 'distance':
                    total_time +=  distance / speed
                    total_distance += distance

                """
                Gets speed of wind against UAV when it is on angle
                Then provides that speed to propulsion power equation
                For each ith segment of the path, with speeds in winds:
                    1) Calculates the time required for that distance
                    2) Gets watthour consumed for that time
                    3) gets milliamphours for defined battery voltage from watthours
                
                Sums each ith segment of milliamphours to how much it would consume for the whole path.
                """
                if cost == 'consumption':
                    wind_speed = wind[terrain_atpoint_index][3]
                    wind_angle = wind[terrain_atpoint_index][4]

                    wind_power_consumption, UAV_speed = getPowerInWind(wind_speed, wind_angle, speed, x_difference, y_difference)
                    seconds = distance / UAV_speed
                    watthour = wind_power_consumption  * (seconds / 3600)
                    milliamphour = watthour / drone.battery_voltage * 1000
                    total_time += seconds
                    total_milliamphour += milliamphour

            if cost == 'distance': movement_matrix[index1].append(total_distance)
            if cost == 'consumption': movement_matrix[index1].append(total_milliamphour)
            time_matrix[index1].append(total_time)
            iteration += 1
    return np.array(movement_matrix), time_matrix, motion_matrix


# Helper function for Power consumption for UAV to hover at cluster points
def hoverPowerConsumptionAtCentroid(centroids, terrain, UAV_hover_time, wind = 0, type = 'nowind'):
    # Function to calculate Euclidean distance
    def euclidean_distance(point1, point2):
        return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(point1, point2)))
    hover_matrix = []

    for index, centroid in enumerate(centroids):
        # Skip first centroid, which is center point
        if index == 0: 
            hover_matrix.append('center_point')  
            continue

        # Function to find the closest terrain point
        closest_point_index = min(range(len(terrain)), key=lambda i: euclidean_distance(terrain[i], centroid))
        
        # Gets wind angle and speed at that closest terrain point
        if type == 'wind': 
            wind_speed = wind[closest_point_index][3]
            # Not using wind angle, because at hovering state dirrection does not matter as much for power consumption
            power_consumption, UAV_speed = getPowerInWind(wind_speed, movement='hover')
        elif type == "nowind": power_consumption = drone.getPropulsionPowerConsumtion(0)

        watthour = power_consumption  * (UAV_hover_time[index - 1] / 3600)
        milliamphour = watthour / drone.battery_voltage * 1000
        hover_matrix.append(milliamphour)
    return hover_matrix
    

# Converts XYZ file each point coordinate values to metered values
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
