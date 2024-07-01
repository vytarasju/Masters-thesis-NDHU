from measurement import IoTDevice, UAV, WPT
from cluster_path import clusterXMeansChargeTime, getMotion, hoverPowerConsumptionAtCentroid
from read_write_data import readCSV
from algorithms import AntColony

"""BEGIN Parameter Defintion"""
# For X-Means
min_hover_WPT = 2 #meters
angle_WPT = 120

# For UAV
UAV_speed = 30 #meters/second
UAV_elevation = 39 #meters
UAV_steps = 20 #steps between points
"""END Parameter Defintion"""

terrain = readCSV('terrain_data.csv')
sensors = readCSV('sensor_coordinates.csv')
wind = readCSV('wind_data.csv')

drone = UAV(terrain=terrain)
iot = IoTDevice()
wpt = WPT()

K_value = 1
sensor_num = len(sensors)
provide_charge = iot.batteryConsumtionGivenTime(0, drone.minimum_operation_time)
print(f'charge all {sensor_num} sensors up to {provide_charge} mAh, to operate for {drone.minimum_operation_time / 60} min')

best_time, best_consumption = [0, float('inf')], [0, float('inf')]

while K_value < sensor_num:
    try: centroids, wpt_area, cluster_charge_time, K_value = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge, K_value)
    except: 
        print('K_value end reached')
        break
    total_cluster_charge_time = uav_hover_time = sum(cluster_charge_time)

    movement_matrix, time_matrix = getMotion(centroids, terrain, UAV_steps, UAV_elevation, 'consumption', wind)
    hover_matrix = hoverPowerConsumptionAtCentroid(centroids, wind, terrain, cluster_charge_time)

    ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)
    aco_path, aco_cost = ant_colony.find_shortest_path()

    wpt_charge_consumption = wpt.chargeConsumptionGivenTime(total_cluster_charge_time)
    hover_charge_consumption = sum(hover_matrix[1:])

    uav_flight_time = 0
    path_solution = aco_path
    for index, path in enumerate(path_solution):
        if path == 0: continue
        time_path = time_matrix[path_solution[index - 1]][path]
        uav_flight_time += time_path

    total_uav_charge_consumption = hover_charge_consumption + aco_cost
    total_uav_operation_time = uav_hover_time + uav_flight_time
    
    if total_uav_operation_time < best_time[1]: 
        best_time[1] = total_uav_operation_time
        best_time[0] = K_value
    if total_uav_charge_consumption < best_consumption[1]: 
        best_consumption[1] = total_uav_charge_consumption
        best_consumption[0] = K_value
    print(aco_path)
    print(f'UAV uses {hover_charge_consumption} mAh in total')
    print(f'This would take {total_uav_operation_time/60} min total operation time')
    K_value += 1

print(f'at K {best_consumption[0]} best consumption {best_consumption[1]} mAh')
print(f'at K {best_time[0]} best time {best_time[1]/60} min')

