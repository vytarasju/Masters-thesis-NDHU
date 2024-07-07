from measurement import IoTDevice, UAV, WPT
from cluster_path import *
from sensors_generation import generateSensorsCentralized
from algorithms import AntColony
from read_write_data import *
import csv
import os
import shutil

"""
Make sure:
    1) Terrain file is in: code/windninja/converted
    2) Wind angle and velocity files are in: code/windninja/wind
Change working directory to a place where to save test results
"""


terrain_name_list = ['LiYu1km.xyz']
# Value meaning: 1st - angle of original wind input; 2nd - wind speed; 3rd - resolution of terrain input
wind_name_extention_list = ['150_10_30m']

# Iterate through multiple terraind and winds
for terrain_name in terrain_name_list:
    for wind_name_extention in wind_name_extention_list:
        wind_velocity_name = terrain_name.split('.')[0] + f'_{wind_name_extention}_vel.asc'
        wind_angle_name = terrain_name.split('.')[0] + f'_{wind_name_extention}_ang.asc'

        wind_parameters_angle = wind_name_extention.split('_')[0]
        wind_parameters_speed = wind_name_extention.split('_')[1]

        working_directory_path = '/home/vytska/thesis/code/csv/'
        working_directory_path += terrain_name.split('.')[0] + '_combined_centralized' + \
            f'_{wind_parameters_angle}deg{wind_parameters_speed}kts' '/'
        working_directory_path_wind = working_directory_path + 'wind'+ '/'
        working_directory_path_nowind = working_directory_path + 'nowind'+ '/'

        # Remove the directory if it exists to reset results for retests
        def createWorkingDirectory(dir_path):
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
            # Create dirrectory
            os.makedirs(dir_path, exist_ok=True)

        createWorkingDirectory(working_directory_path)
        createWorkingDirectory(working_directory_path_wind)
        createWorkingDirectory(working_directory_path_nowind)

        
        """BEGIN Motion and WPT Parameter Defintion"""
        # For X-Means
        min_hover_WPT = 2 #meters
        angle_WPT = 120 #degrees

        # For UAV
        UAV_speed = 30 #meters/second
        UAV_elevation = 39 #meters
        UAV_steps = 20 #steps between points
        """END Parameter Defintion"""

        """BEGIN Sensor and Terrain Parameter Definition"""
        terrain_resolution = 30 #according terrain data used

        sensors_num = 500
        density = 0.5
        starting_points = 1

        # For iteration process
        density_increment = 0.5
        density_limit = 10

        starting_points_increment = 1
        starting_points_limit = 20
        """END Sensor and Terrain Parameter Definition"""

        """BEGIN Terrain, Wind and Devices Definition"""
        terrain = readTerrainXYZ(terrain_name)
        terrain = convertXYZtoMeters(terrain)
        wind = readWindASCWindninja(terrain, wind_velocity_name, wind_angle_name)

        # Flatten wind list, to match CSV layout used in other parts of the project
        wind = [list(list_item[0]) + list_item[1:] for list_item in wind]

        drone = UAV()
        iot = IoTDevice()
        wpt = WPT()
        """END Terrain, Wind and Devices Definition"""

        # Save Terrain and Wind CSV in the testing directory for future reference
        with open(working_directory_path + 'terrain_data.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['terrain_x', 'terrain_y', 'terrain_z'])  # Write header
                for point in terrain:
                        writer.writerow([point[0], point[1], point[2]])
        with open(working_directory_path + 'wind_data.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['terrain_x', 'terrain_y', 'terrain_z', 'wind_velocity', 'wind_angle'])  # Write header
                for point in wind:
                        writer.writerow([point[0], point[1], point[2], point[3], point[4]])

        # Helper function for writing and printing iteration end
        def write_print(text):
            file.write(text + '\n')
            print(text)

        def dictionaryValueReset(best_variables, type):
            values = {
                "k": best_variables["k"],
                "time": best_variables["time"],
                "consumption": best_variables["consumption"].copy(),
                "path": best_variables["path"].copy(),
                "clusters": best_variables["clusters"].copy(),
                "motion": best_variables["motion"].copy(),
                "movement": best_variables["movement"].copy(),
            }
            if type == 'nowind': values["movement_wind"] = best_variables["movement"].copy()
            return values

        # Best performing variables
        best_variables = {
            "k": 0,
            "time": float('inf'),
            "consumption": [float('inf'), float('inf'), float('inf')],
            "path": [],
            "clusters": [],
            "motion": [],
            "movement": [],
            "wpt-consumption": float('inf')
        }
        best_wind = best_variables.copy()
        best_nowind = best_variables.copy()

        # Helper Function: Saves results in specific test run directory
        # Usage: In end iteration function
        def saveResults(dictionary, dir_path, type = 'none'):
            # K_value max reached, output best solution, if it was found
            if dictionary["time"] !=  float('inf') and dictionary["consumption"][0] !=  float('inf'):
                write_print(f'BEST TOTAL SOLUTION WAS FOUND')
                write_print(f'consumption: {dictionary["consumption"][0]:.2f} mAh')
                write_print(f'K{dictionary["k"]} time: {(dictionary["time"]/60):.2f} min')
                
                # Save sensors, clusters results for best solutions at the specified density and starting_points
                solution_working_directory_path = dir_path + f'D{density}SP{starting_points}/'
                os.makedirs(solution_working_directory_path, exist_ok=True)
                with open(solution_working_directory_path + 'sensors.csv', mode='w') as solution_file:
                    writer = csv.writer(solution_file)
                    writer.writerow(['sensor_x', 'sensor_y', 'sensor_z'])  # Write header
                    for sensor in sensors:
                        writer.writerow([sensor[0], sensor[1], sensor[2]])
                with open(solution_working_directory_path + 'clusters.csv', mode='w') as solution_file:
                    writer = csv.writer(solution_file)
                    writer.writerow(['cluster_x', 'cluster_y', 'cluster_z'])  # Write header
                    for cluster in dictionary["clusters"]:
                            writer.writerow([cluster[0], cluster[1], cluster[2]])
                with open(solution_working_directory_path + 'motion.csv', mode='w') as solution_file:
                    writer = csv.writer(solution_file)
                    for motion in dictionary["motion"]:
                            writer.writerow(motion)
                with open(solution_working_directory_path + 'path.txt', mode='w') as solution_file:
                    solution_file.write(f'{dictionary["path"]}')
                with open(solution_working_directory_path + 'solution_results.txt', mode='w') as solution_file:
                    solution_file.write(f'Charge all {sensors_num} sensors up to {provide_charge:.2f} mAh, to operate for {(drone.minimum_operation_time/60):.2f} min\n')
                    solution_file.write(f'Total UAV charge consumption: {dictionary["consumption"][0]:.2f} mAh\n')
                    solution_file.write(f'UAV hover: {dictionary["consumption"][1]:.2f} mAh\n')
                    solution_file.write(f'UAV flight: {dictionary["consumption"][2]:.2f} mAh\n')
                    solution_file.write(f'Total UAV time taken: {(dictionary["time"]/60):.2f} min\n')
                    solution_file.write(f'Total WPT charge consumption: {(dictionary["wpt-consumption"]):.2f} mAh\n')
                with open(solution_working_directory_path + 'movement.csv', mode='w') as solution_file:
                    writer = csv.writer(solution_file)
                    for line in dictionary["movement"]:
                            writer.writerow(line)
                if type == 'nowind':
                    with open(solution_working_directory_path + 'movement_wind.csv', mode='w') as solution_file:
                        writer = csv.writer(solution_file)
                        for line in dictionary["movement_wind"]:
                                writer.writerow(line)
            else:
                write_print(f'NO TOTAL SOLUTION WAS FOUND')

        # Usage: end iteration after all K values have been checked
        def endIteration(K_value, starting_points, K_ceiling_counter, best_wind, best_nowind):
            # Print header information
            print(f'Terrain: {terrain_name}; Wind{wind_name_extention}')
            write_print(f'D{density}SP{starting_points}: At {K_value}K limit reached')

            # Save wind and nowind results
            write_print(f'WIND results')
            saveResults(dictionary=best_wind, dir_path=working_directory_path_wind, type = 'wind')
            write_print(f'NOWIND results')
            saveResults(dictionary=best_nowind, dir_path=working_directory_path_nowind, type = 'nowind')
            write_print(f'\n')

            # Reset variables
            K_value = 1
            K_ceiling_counter = 0
            best_wind = dictionaryValueReset(best_variables, 'wind')
            best_nowind = dictionaryValueReset(best_variables, 'nowind')
            
            continue_loop = True
            # If starting_points still can be increased, then increase it, if not then increase density and reset starting_points
            if starting_points < starting_points_limit: 
                starting_points += starting_points_increment
                # Regenerate sensors at higher starting_points
                sensors = generateSensorsCentralized(terrain, sensors_num, density, starting_points, terrain_resolution)
            elif starting_points >= starting_points_limit:
                continue_loop = False
                sensors = 0
            return K_value, K_ceiling_counter, best_wind, best_nowind, continue_loop, starting_points, sensors

        # Runs through the whole solution finding process
        # Usage: after sensors and clusters found for each iteration
        def runTest(K_ceiling_counter, dictionary, movement_wind = 0, type = 'none'):
            # Find all possilbe paths the UAV can take and get power consumption needed at eaceh hovering point
            if type == 'wind':
                print(f'WIND D{density} SP{starting_points} K{K_value}')
                getMotionCost = 'consumption'
                hoverCostType = 'wind'
            elif type == 'nowind':
                print(f'NOWIND D{density} SP{starting_points} K{K_value}')
                getMotionCost = 'distance'
                hoverCostType = 'nowind'

            movement_matrix, time_matrix, motion_matrix = getMotion(clusters, terrain, UAV_steps, UAV_elevation, getMotionCost, wind)
            hover_matrix = hoverPowerConsumptionAtCentroid(clusters, terrain, cluster_charge_time, wind, type = hoverCostType)

            # Find path solution for UAV
            ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)
            aco_path, aco_cost = ant_colony.find_shortest_path()
            path_solution = aco_path
            if type == "nowind": aco_cost, movement_matrix = drone.convertDistancetoMeasurements(aco_cost, movement_matrix, type='milliamphours')
            flight_consumption = aco_cost

            # Find WPT and hovering charge consumption
            total_cluster_charge_time = uav_hover_time = sum(cluster_charge_time)
            wpt_charge_consumption = wpt.chargeConsumptionGivenTime(total_cluster_charge_time)
            hover_charge_consumption = sum(hover_matrix[1:])

            # Find total UAV flight time
            uav_flight_time = 0
            for index, path in enumerate(path_solution):
                if path == 0: continue
                time_path = time_matrix[path_solution[index - 1]][path]
                uav_flight_time += time_path

            # Sum up all UAV charge consumption and operation time variables
            total_uav_charge_consumption = hover_charge_consumption + flight_consumption
            total_uav_operation_time = uav_hover_time + uav_flight_time
            
            # Output current computations
            print(f'UAV: Path {aco_path}')
            print(f'UAV: 1) Charge {hover_charge_consumption:.2f} mAh 2) Time {(total_uav_operation_time/60):.2f} min')

            continue_loop = True
            # Check if the current solution fits UAV operation time and charge amount requirements
            if total_uav_operation_time > drone.minimum_operation_time:
                print('Not Solution: IOT Devices charged less than UAV operation time \n')
            elif hover_charge_consumption > drone.battery_capacity:
                print('Not Solution: UAV battery exceeded \n')
            else: 
                print('Solution \n')
                # If current solution time and charging is smaller, then record this as best solution
                if (total_uav_operation_time < dictionary["time"]) and (total_uav_charge_consumption < dictionary["consumption"][0]):
                    dictionary.update({
                        "k": K_value,
                        "time": total_uav_operation_time,
                        "consumption": [total_uav_charge_consumption, hover_charge_consumption, flight_consumption],
                        "path": path_solution,
                        "clusters": clusters,
                        "motion": motion_matrix,
                        "movement": movement_matrix,
                        "wpt-consumption": wpt_charge_consumption
                    })
                    if type == 'nowind': dictionary["movement_wind"] = movement_wind
                    K_ceiling_counter = 0
                else: 
                    K_ceiling_counter += 1
                    if K_ceiling_counter == K_ceiling_limit: continue_loop = False   
            if type == 'wind': return  dictionary, K_ceiling_counter, continue_loop, movement_matrix
            if type == 'nowind': return  dictionary, K_ceiling_counter, continue_loop

        K_value = 1
        K_ceiling_counter, K_ceiling_limit = 0, 5
        starting_points_init = starting_points
        provide_charge = iot.batteryConsumtionGivenTime(0, drone.minimum_operation_time)
        # Write and Print all of the log text to have backup for later and to keep track on live iteration progression
        with open(working_directory_path + 'test_results.txt', mode='w') as file:
            write_print(f'Charge {sensors_num} sensors up to {provide_charge:.2f} mAh, to operate for {(drone.minimum_operation_time / 60):.2f} min')
            write_print(f'\n')

            # Increase density and starting points
            # To get best K_value at different distribution levels
            while density <= density_limit:
                # Init first sensors after density change
                starting_points = starting_points_init
                sensors = generateSensorsCentralized(terrain, sensors_num, density, starting_points, terrain_resolution)

                # Rest dictionary values
                best_wind = dictionaryValueReset(best_variables, 'wind')
                best_nowind = dictionaryValueReset(best_variables, 'nowind')

                # Finding best K_value
                while K_value < sensors_num:
                    # Try clustering, if it fails - K_Value max reached, increase starting_points
                    # If starting_points reached limit, increase density and start at the begining with starting_points
                    try: clusters, wpt_area, cluster_charge_time, K_value = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge, K_value)
                    except:
                        write_print('ITEREND: X-Means exception reached')
                        K_value, K_ceiling_counter, best_wind, best_nowind, continue_density_loop, starting_points, sensors = endIteration(K_value, starting_points, K_ceiling_counter, best_wind, best_nowind)
                        if continue_density_loop: continue
                        elif not continue_density_loop: break

                    # run testruns for wind and nowind solutions
                    best_wind, K_ceiling_counter, continue_sp_loop_wind, movement_wind = runTest(K_ceiling_counter, best_wind, type="wind")
                    best_nowind, K_ceiling_counter, continue_sp_loop_nowind = runTest(K_ceiling_counter, best_nowind, movement_wind, type="nowind")

                    if not continue_sp_loop_wind or not continue_sp_loop_nowind:
                        if not continue_sp_loop_wind and not continue_sp_loop_nowind: write_print('ITEREND: K ceiling reached by wind and nowind')
                        elif not continue_sp_loop_wind: write_print('ITEREND: K ceiling reached by wind')
                        elif not continue_sp_loop_nowind: write_print('ITEREND: K ceiling reached by nowind')
                        K_value, K_ceiling_counter, best_wind, best_nowind, continue_density_loop, starting_points, sensors = endIteration(K_value, starting_points, K_ceiling_counter, best_wind, best_nowind)
                        if continue_density_loop == True: continue
                        elif continue_density_loop == False: break
                    K_value += 1
                # starting_points_limit reached, expand on density and try again
                density += density_increment

