from measurement import IoTDevice, UAV, WPT
from cluster_path import *
from sensors_generation import generateSensorsUniform
from algorithms import AntColony, GeneticAlgorithm, BlackHoleAlgorithm
from read_write_data import *
import csv
import os
import shutil
import copy

"""
Make sure:
    1) Terrain file is in: code/windninja/converted
    2) Wind angle and velocity files are in: code/windninja/wind
Change working directory to a place where to save test results
"""
#'TongMen1km.xyz', 'LiYu1km.xyz', 'YanZiKou1km.xyz', 'FuXing1km.xyz', 'ZiYouLi1km.xyz', 'DongHwa1km.xyz'
terrain_name_list = ['TongMen1km.xyz', 'LiYu1km.xyz']
wind_name_extention_list = ['150_7_30m', '250_7_30m']


# Iterate through multiple terraind and winds
for terrain_name in terrain_name_list:
    for wind_name_extention in wind_name_extention_list:
        wind_velocity_name = terrain_name.split('.')[0] + f'_{wind_name_extention}_vel.asc'
        wind_angle_name = terrain_name.split('.')[0] + f'_{wind_name_extention}_ang.asc'

        wind_parameters_angle = wind_name_extention.split('_')[0]
        wind_parameters_speed = wind_name_extention.split('_')[1]

        working_directory_path = '/home/vytska/thesis/code/csv/2weeks-uniform/'
        working_directory_path += terrain_name.split('.')[0] + '_combined_uniform' + \
            f'_{wind_parameters_angle}deg{wind_parameters_speed}kts' '/'
        
        aco_working_directory_path_wind = working_directory_path + 'ACO_wind'+ '/'
        aco_working_directory_path_nowind = working_directory_path + 'ACO_nowind'+ '/'
        aco_working_directory_path_nowind_inwind = working_directory_path + 'ACO_nowind-inwind'+ '/'

        ga_working_directory_path_wind = working_directory_path + 'GA_wind'+ '/'
        ga_working_directory_path_nowind = working_directory_path + 'GA_nowind'+ '/'
        ga_working_directory_path_nowind_inwind = working_directory_path + 'GA_nowind-inwind'+ '/'

        bh_working_directory_path_wind = working_directory_path + 'BH_wind'+ '/'
        bh_working_directory_path_nowind = working_directory_path + 'BH_nowind'+ '/'
        bh_working_directory_path_nowind_inwind = working_directory_path + 'BH_nowind-inwind'+ '/'

        # Remove the directory if it exists to reset results for retests
        def createWorkingDirectory(dir_path):
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
            # Create dirrectory
            os.makedirs(dir_path, exist_ok=True)

        createWorkingDirectory(working_directory_path)
        
        createWorkingDirectory(aco_working_directory_path_wind)
        createWorkingDirectory(aco_working_directory_path_nowind)
        createWorkingDirectory(aco_working_directory_path_nowind_inwind)

        createWorkingDirectory(ga_working_directory_path_wind)
        createWorkingDirectory(ga_working_directory_path_nowind)
        createWorkingDirectory(ga_working_directory_path_nowind_inwind)

        createWorkingDirectory(bh_working_directory_path_wind)
        createWorkingDirectory(bh_working_directory_path_nowind)
        createWorkingDirectory(bh_working_directory_path_nowind_inwind)
        
        """BEGIN Motion and WPT Parameter Defintion"""
        # For X-Means
        min_hover_WPT = 2 #meters
        angle_WPT = 120 #degrees

        # For UAV
        UAV_speed = 30 #meters/second
        UAV_elevation = 30 #meters
        UAV_steps = 20 #steps between points
        """END Parameter Defintion"""

        """BEGIN Iteration Parameter Definition"""
        # For solution limiter
        limit_type = 'nolimit'

        # For sensor amount, limiter, increment
        sensors_num = 2
        sensors_num_limit = 50
        sensors_num_increment = 1
        """END Iteration Parameter Definition"""

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

        # Helper function for writing and printing
        def write_print(text):
            file.write(text + '\n')
            print(text)

        def dictionaryValueReset(best_variables):
            return copy.deepcopy(best_variables)
        
        # Best performing variable dictionary
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

        # Helper Function: Saves results in specific test run directory
        # Usage: In end iteration function
        def saveResults(dictionary, dir_path):
            # K_value max reached, output best solution, if it was found
            if dictionary["time"] !=  float('inf') and dictionary["consumption"][0] !=  float('inf'):
                write_print(f'TEST!BEST TOTAL SOLUTION WAS FOUND')
                write_print(f'TEST!CONSUMPTION: {dictionary["consumption"][0]:.2f} mAh')
                write_print(f'TEST!K{dictionary["k"]} time: {(dictionary["time"]/60):.2f} min')
                
                # Save sensors, clusters results for best solutions at the specified density and starting_points
                solution_working_directory_path = dir_path + f'SN{sensors_num}/'
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
            else:
                write_print(f'TEST!NO TOTAL SOLUTION WAS FOUND')

        # Usage: end iteration after all K values have been checked
        def endIteration(K_value, sensors_num, best_wind, best_nowind, best_nowind_inwind, type=''):
            # Print header information
            print(f'{type}TEST!TERRAIN: {terrain_name}; Wind{wind_name_extention}')
            write_print(f'{type}TEST!SN{sensors_num}: At {K_value}K limit reached')

            # Save results
            if type == 'ACO':
                write_print(f'{type}TEST!WIND results')
                saveResults(dictionary=best_wind, dir_path=aco_working_directory_path_wind)
                write_print(f'{type}TEST!NOWIND results')
                saveResults(dictionary=best_nowind, dir_path=aco_working_directory_path_nowind)
                write_print(f'{type}TEST!NOWIND-INWIND results')
                saveResults(dictionary=best_nowind_inwind, dir_path=aco_working_directory_path_nowind_inwind)
            elif type == 'GA':
                write_print(f'{type}TEST!WIND results')
                saveResults(dictionary=best_wind, dir_path=ga_working_directory_path_wind)
                write_print(f'{type}TEST!NOWIND results')
                saveResults(dictionary=best_nowind, dir_path=ga_working_directory_path_nowind)
                write_print(f'{type}TEST!NOWIND-INWIND results')
                saveResults(dictionary=best_nowind_inwind, dir_path=ga_working_directory_path_nowind_inwind)
            elif type == 'BH':
                write_print(f'{type}TEST!WIND results')
                saveResults(dictionary=best_wind, dir_path=bh_working_directory_path_wind)
                write_print(f'{type}TEST!NOWIND results')
                saveResults(dictionary=best_nowind, dir_path=bh_working_directory_path_nowind)
                write_print(f'{type}TEST!NOWIND-INWIND results')
                saveResults(dictionary=best_nowind_inwind, dir_path=bh_working_directory_path_nowind_inwind)

            write_print(f'\n')

        # Runs through the whole solution finding process
        # Usage: after sensors and clusters found for each iteration
        def runTest(K_ceiling_counter, dictionary, limit_type='UAV', test_type='NA', path_algorithm='ACO'):
            # Find all possilbe paths the UAV can take and get power consumption needed at eaceh hovering point
            print(f'{path_algorithm}TEST!{test_type} SN{sensors_num} K{K_value}')
            if test_type == 'wind':
                getMotionCost = 'consumption'
                hoverCostType = 'wind'
            elif test_type == 'nowind':
                getMotionCost = 'distance'
                hoverCostType = 'nowind'
            elif test_type == 'nowind-inwind':
                getMotionCost = 'distance'
                hoverCostType = 'wind'

            # Handle limit_type parameter
            if limit_type not in ['UAV', 'nolimit']:
                raise ValueError("limit_type must be either 'UAV' or 'nolimit'")

            if test_type != 'nowind-inwind': 
                movement_matrix, time_matrix, motion_matrix = getMotion(clusters, terrain, UAV_steps, UAV_elevation, getMotionCost, wind)
                hover_matrix = hoverPowerConsumptionAtCentroid(clusters, terrain, cluster_charge_time, wind, type = hoverCostType)
                if path_algorithm == 'ACO':
                    ant_colony = AntColony(movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)
                    path_solution, flight_consumption = ant_colony.find_shortest_path()
                if path_algorithm == 'GA':
                    genetic_algorithm = GeneticAlgorithm(movement_matrix, population_size=200, mutation_rate=0.01, num_generations=1000)
                    path_solution, flight_consumption = genetic_algorithm.find_shortest_path()
                if path_algorithm == 'BH':
                    blackhole_algorithm = BlackHoleAlgorithm(movement_matrix, num_stars=100, num_iterations=3000)
                    path_solution, flight_consumption = blackhole_algorithm.find_shortest_path()
                if test_type == 'nowind': flight_consumption, movement_matrix = drone.convertDistancetoMeasurements(flight_consumption, movement_matrix, type='milliamphours')
            
            elif test_type == 'nowind-inwind':
                nowind_movement_matrix, time_matrix, motion_matrix = getMotion(clusters, terrain, UAV_steps, UAV_elevation, getMotionCost, wind)
                wind_movement_matrix, time_matrix, __ = getMotion(clusters, terrain, UAV_steps, UAV_elevation, 'consumption', wind)
                hover_matrix = hoverPowerConsumptionAtCentroid(clusters, terrain, cluster_charge_time, wind, type = hoverCostType)
                if path_algorithm == 'ACO':
                    ant_colony = AntColony(nowind_movement_matrix, num_ants=80, num_iterations=50, evaporation_rate=0.5, alpha=1, beta=1)
                    path_solution, ___ = ant_colony.find_shortest_path()
                if path_algorithm == 'GA':
                    genetic_algorithm = GeneticAlgorithm(nowind_movement_matrix, population_size=200, mutation_rate=0.01, num_generations=1000)
                    path_solution, ___ = genetic_algorithm.find_shortest_path()
                if path_algorithm == 'BH':
                    blackhole_algorithm = BlackHoleAlgorithm(nowind_movement_matrix, num_stars=100, num_iterations=3000)
                    path_solution, flight_consumption = blackhole_algorithm.find_shortest_path()
                total_path_cost = 0
                for index, destination in enumerate(path_solution):
                    if index > 0: 
                        actual_path_cost = wind_movement_matrix[last_destination][destination]
                        total_path_cost += actual_path_cost
                    last_destination = destination
                flight_consumption = total_path_cost
                movement_matrix = wind_movement_matrix

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
            print(f'TEST!UAV: Path {path_solution}')
            print(f'TEST!UAV: 1) Charge {hover_charge_consumption:.2f} mAh 2) Time {(total_uav_operation_time/60):.2f} min')

            continue_loop = True
            is_solution = True
            # Check if the current solution fits UAV operation time and charge amount requirements
            if limit_type == 'UAV':
                if total_uav_operation_time > drone.minimum_operation_time:
                    print('TEST!NOTSOLUTION: IOT Devices charged less than UAV operation time \n')
                    is_solution == False
                elif total_uav_charge_consumption > drone.battery_capacity:
                    print('TEST!NOTSOLUTION: UAV battery exceeded \n')
                    is_solution == False
            # If current solution time and charging is smaller, then record this as best solution
            if is_solution and (total_uav_operation_time < dictionary["time"]) and (total_uav_charge_consumption < dictionary["consumption"][0]):
                print('TEST!SOLUTION \n')
                new_dictionary = copy.deepcopy(dictionary)
                new_dictionary.update({
                    "k": K_value,
                    "time": total_uav_operation_time,
                    "consumption": [total_uav_charge_consumption, hover_charge_consumption, flight_consumption],
                    "path": path_solution,
                    "clusters": clusters,
                    "motion": motion_matrix,
                    "movement": movement_matrix,
                    "wpt-consumption": wpt_charge_consumption
                })
                dictionary = copy.deepcopy(new_dictionary)
                K_ceiling_counter = 0
            else:
                print('TEST!NOTSOLUTION: Previous results are better \n')
                K_ceiling_counter += 1
                if K_ceiling_counter == K_ceiling_limit: continue_loop = False   
            return  dictionary, K_ceiling_counter, continue_loop
        
        def processDuplicates(data):
            seen = {}
            found_Duplicate = False

            for index, entry in enumerate(data):
                entry_tuple = tuple(entry)
                if entry_tuple in seen:
                    write_print(f'TEST!XMeans_ERROR: Duplicate cluster found with K{K_value}, trying again')
                    found_Duplicate = True
                    break
                else:
                    seen[entry_tuple] = index
            return found_Duplicate

        K_ceiling_limit = 5
        K_value_init = 2
        provide_charge = iot.batteryConsumtionGivenTime(0, drone.minimum_operation_time)
        # Write and Print all of the log text to have backup for later and to keep track on live iteration progression
        with open(working_directory_path + 'test_results.txt', mode='w') as file:
            write_print(f'Charge at most {sensors_num_limit} sensors up to {provide_charge:.2f} mAh, to operate for {(drone.minimum_operation_time / 60):.2f} min')
            write_print(f'\n')

            # Increase density and starting points
            # To get best K_value at different distribution levels
            while sensors_num <= sensors_num_limit:
                # Init first sensors after density change
                K_value = K_value_init
                K_ceiling_counter = 0
                limit_value = float('inf')
                counter_reset_XMeans = 0
                XMeans_exception = False
                XMeans_iterend = False
                sensors = generateSensorsUniform(terrain, sensors_num)

                # Rest dictionary values
                aco_best_wind = dictionaryValueReset(best_variables)
                aco_best_nowind = dictionaryValueReset(best_variables)
                aco_best_nowind_inwind = dictionaryValueReset(best_variables)
                aco_continue_loop_wind, aco_continue_loop_nowind, aco_continue_loop_nowind_inwind = True, True, True

                ga_best_wind = dictionaryValueReset(best_variables)
                ga_best_nowind = dictionaryValueReset(best_variables)
                ga_best_nowind_inwind = dictionaryValueReset(best_variables)
                ga_continue_loop_wind, ga_continue_loop_nowind, ga_continue_loop_nowind_inwind = True, True, True

                bh_best_wind = dictionaryValueReset(best_variables)
                bh_best_nowind = dictionaryValueReset(best_variables)
                bh_best_nowind_inwind = dictionaryValueReset(best_variables)
                bh_continue_loop_wind, bh_continue_loop_nowind, bh_continue_loop_nowind_inwind = True, True, True

                # Finding best K_value
                while K_value <= sensors_num:
                    temp_K_value = K_value
                    XMeans_exception = False
                    try: XMeans_result = clusterXMeansChargeTime(terrain, sensors, angle_WPT, min_hover_WPT, provide_charge, limit_value, limit_type, K_value)
                    except: 
                        write_print(f'TEST!XMeans_ERROR: At K{temp_K_value} EXCEPTION')
                        XMeans_exception = True

                    if isinstance(XMeans_result, tuple): clusters, wpt_area, cluster_charge_time, limit_value, K_value = XMeans_result
                    else:
                        if XMeans_result == 'K_Ceiling' or XMeans_result == 'K_Value': 
                            if XMeans_result == 'K_Ceiling': write_print(f'TEST!ITEREND: At K{temp_K_value} X-Means: reached K_Ceiling')
                            elif XMeans_result == 'K_Value': write_print(f'TEST!ITEREND: At K{temp_K_value} X-Means: K_Value reached sensors_num')
                            endIteration(K_value - 1, sensors_num, aco_best_wind, aco_best_nowind, aco_best_nowind_inwind, 'ACO')
                            endIteration(K_value - 1, sensors_num, ga_best_wind, ga_best_nowind, ga_best_nowind_inwind, 'GA')
                            endIteration(K_value - 1, sensors_num, bh_best_wind, bh_best_nowind, bh_best_nowind_inwind, 'BH')
                            XMeans_iterend = True
                            break
                        elif XMeans_result == 'K_Floor': 
                            write_print(f'TEST!XMeans_ERROR: At K{temp_K_value} reached K_Floor')
                            write_print(f'TEST!RESETITER: reseting K_value and regenerating sensors')

                            sensors = generateSensorsUniform(terrain, sensors_num)
                            K_value = K_value_init
                            limit_value = float('inf')
                            # Rest dictionary values
                            aco_best_wind = dictionaryValueReset(best_variables)
                            aco_best_nowind = dictionaryValueReset(best_variables)
                            aco_best_nowind_inwind = dictionaryValueReset(best_variables)
                            aco_continue_loop_wind, aco_continue_loop_nowind, aco_continue_loop_nowind_inwind = True, True, True

                            ga_best_wind = dictionaryValueReset(best_variables)
                            ga_best_nowind = dictionaryValueReset(best_variables)
                            ga_best_nowind_inwind = dictionaryValueReset(best_variables)
                            ga_continue_loop_wind, ga_continue_loop_nowind, ga_continue_loop_nowind_inwind = True, True, True

                            bh_best_wind = dictionaryValueReset(best_variables)
                            bh_best_nowind = dictionaryValueReset(best_variables)
                            bh_best_nowind_inwind = dictionaryValueReset(best_variables)
                            bh_continue_loop_wind, bh_continue_loop_nowind, bh_continue_loop_nowind_inwind = True, True, True
                    
                    match_err_XMeans = False
                    # Check if cluster number matches K_value
                    if (len(clusters) - 1) != K_value:
                        reset_XMeans = True
                        match_err_XMeans = True
                        write_print(f'TEST!XMeans_ERROR: At K{temp_K_value} clusters number does not match K')

                    reset_XMeans = processDuplicates(clusters)
                    if reset_XMeans:
                        K_value = temp_K_value
                        counter_reset_XMeans += 1
                        if counter_reset_XMeans == 3:
                            #If match error flag raised, then dont print this message
                            if not match_err_XMeans: write_print(f'TEST!XMeans_ERROR: At K{temp_K_value} reached duplicate limit')
                            write_print(f'TEST!RESETITER: reseting K_value and regenerating sensors')

                            sensors = generateSensorsUniform(terrain, sensors_num)
                            K_value = K_value_init
                            limit_value = float('inf')
                            counter_reset_XMeans = 0
                            # Rest dictionary values
                            aco_best_wind = dictionaryValueReset(best_variables)
                            aco_best_nowind = dictionaryValueReset(best_variables)
                            aco_best_nowind_inwind = dictionaryValueReset(best_variables)
                            aco_continue_loop_wind, aco_continue_loop_nowind, aco_continue_loop_nowind_inwind = True, True, True

                            ga_best_wind = dictionaryValueReset(best_variables)
                            ga_best_nowind = dictionaryValueReset(best_variables)
                            ga_best_nowind_inwind = dictionaryValueReset(best_variables)
                            ga_continue_loop_wind, ga_continue_loop_nowind, ga_continue_loop_nowind_inwind = True, True, True

                            bh_best_wind = dictionaryValueReset(best_variables)
                            bh_best_nowind = dictionaryValueReset(best_variables)
                            bh_best_nowind_inwind = dictionaryValueReset(best_variables)
                            bh_continue_loop_wind, bh_continue_loop_nowind, bh_continue_loop_nowind_inwind = True, True, True
                        continue
                    else: 
                        counter_reset_XMeans = 0
                        if XMeans_exception:
                            write_print(f'TEST!XMeans_ERROR: At K{temp_K_value} EXCEPTION AND NOT DUPLICATE')
                            write_print(f'TEST!RESETITER: reseting K_value and regenerating sensors')

                            sensors = generateSensorsUniform(terrain, sensors_num)
                            K_value = K_value_init
                            limit_value = float('inf')
                            # Rest dictionary values
                            aco_best_wind = dictionaryValueReset(best_variables)
                            aco_best_nowind = dictionaryValueReset(best_variables)
                            aco_best_nowind_inwind = dictionaryValueReset(best_variables)
                            aco_continue_loop_wind, aco_continue_loop_nowind, aco_continue_loop_nowind_inwind = True, True, True

                            ga_best_wind = dictionaryValueReset(best_variables)
                            ga_best_nowind = dictionaryValueReset(best_variables)
                            ga_best_nowind_inwind = dictionaryValueReset(best_variables)
                            ga_continue_loop_wind, ga_continue_loop_nowind, ga_continue_loop_nowind_inwind = True, True, True

                            bh_best_wind = dictionaryValueReset(best_variables)
                            bh_best_nowind = dictionaryValueReset(best_variables)
                            bh_best_nowind_inwind = dictionaryValueReset(best_variables)
                            bh_continue_loop_wind, bh_continue_loop_nowind, bh_continue_loop_nowind_inwind = True, True, True
                            continue

                    # run testruns for wind and nowind solutions
                    if aco_continue_loop_wind: aco_best_wind, K_ceiling_counter, aco_continue_loop_wind = \
                        runTest(K_ceiling_counter, aco_best_wind, limit_type, "wind", "ACO")
                    elif not aco_continue_loop_wind: print(f'ACOTEST!WIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if aco_continue_loop_nowind: aco_best_nowind, K_ceiling_counter, aco_continue_loop_nowind = \
                        runTest(K_ceiling_counter, aco_best_nowind, limit_type, "nowind", "ACO")
                    elif not aco_continue_loop_nowind: print(f'ACOTEST!NOWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if aco_continue_loop_nowind_inwind: aco_best_nowind_inwind, K_ceiling_counter, aco_continue_loop_nowind_inwind = \
                        runTest(K_ceiling_counter, aco_best_nowind_inwind, limit_type, "nowind-inwind", "ACO")
                    elif not aco_continue_loop_nowind_inwind: print(f'ACOTEST!NOWIND-INWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')

                    if ga_continue_loop_wind: ga_best_wind, K_ceiling_counter, ga_continue_loop_wind = \
                        runTest(K_ceiling_counter, ga_best_wind, limit_type, "wind", "GA")
                    elif not ga_continue_loop_wind: print(f'ACOTEST!WIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if ga_continue_loop_nowind: ga_best_nowind, K_ceiling_counter, ga_continue_loop_nowind = \
                        runTest(K_ceiling_counter, ga_best_nowind, limit_type, "nowind", "GA")
                    elif not ga_continue_loop_nowind: print(f'ACOTEST!NOWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if ga_continue_loop_nowind_inwind: ga_best_nowind_inwind, K_ceiling_counter, ga_continue_loop_nowind_inwind = \
                        runTest(K_ceiling_counter, ga_best_nowind_inwind, limit_type, "nowind-inwind", "GA")
                    elif not ga_continue_loop_nowind_inwind: print(f'ACOTEST!NOWIND-INWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')

                    if bh_continue_loop_wind: bh_best_wind, K_ceiling_counter, bh_continue_loop_wind = \
                        runTest(K_ceiling_counter, bh_best_wind, limit_type, "wind", "BH")
                    elif not bh_continue_loop_wind: print(f'ACOTEST!WIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if bh_continue_loop_nowind: bh_best_nowind, K_ceiling_counter, bh_continue_loop_nowind = \
                        runTest(K_ceiling_counter, bh_best_nowind, limit_type, "nowind", "BH")
                    elif not bh_continue_loop_nowind: print(f'ACOTEST!NOWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')
                    if bh_continue_loop_nowind_inwind: bh_best_nowind_inwind, K_ceiling_counter, bh_continue_loop_nowind_inwind = \
                        runTest(K_ceiling_counter, bh_best_nowind_inwind, limit_type, "nowind-inwind", "BH")
                    elif not bh_continue_loop_nowind_inwind: print(f'ACOTEST!NOWIND-INWIND SN{sensors_num} K{K_value} \n K_Ceiling Reached')

                    K_value += 1

                if XMeans_iterend == False:
                    write_print('TEST!ITEREND: K_value reached sensors_num')
                    endIteration(K_value - 1, sensors_num, aco_best_wind, aco_best_nowind, aco_best_nowind_inwind, 'ACO')
                    endIteration(K_value - 1, sensors_num, ga_best_wind, ga_best_nowind, ga_best_nowind_inwind, 'GA')
                    endIteration(K_value - 1, sensors_num, bh_best_wind, bh_best_nowind, bh_best_nowind_inwind, 'BH')
                
                sensors_num += sensors_num_increment
