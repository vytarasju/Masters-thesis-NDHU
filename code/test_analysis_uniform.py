from read_write_data import *
from plot import *
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import re

#'TongMen1km', 'LiYu1km', 'YanZiKou1km', 'FuXing1km', 'ZiYouLi1km', 'DongHwa1km'
test_name_list = ['TongMen1km', 'LiYu1km']
# wind_name_extention_list_3kts = ['60deg3kts']
wind_name_extention_list_7kts = ['150deg7kts', '250deg7kts']
alg_name_list = ['ACO', 'GA', 'BH']
csv_path = '/home/vytska/thesis/code/csv/multi-alg-finished/'


def main(test_name, wind_name_extention):
    test_directory = test_name + '_combined_uniform_' + wind_name_extention + '/'
    readCSV_path = test_directory

    def readMovementFile(directory_path):
        movement = []
        with open(directory_path + 'movement.csv', 'r') as file:
            reader = csv.reader(file)
            for line in reader: movement.append(line)
            return movement

    def readPathFile(directory_path):
        with open(directory_path + 'path.txt', 'r') as file:
            for line in file:
                line = line.split(', ')
                trans_table = str.maketrans('', '', '[]')
                line = [int(item.translate(trans_table)) for item in line]
                path = line
        return path

    def readSolutionResults(directory_path):
        UAV_consumption = []
        UAV_time = 0
        with open(directory_path + 'solution_results.txt', 'r') as file:
            for index, line in enumerate(file):
                if index > 0 and index < 4: UAV_consumption.append(line.split(':')[1].split(' ')[1])
                if index == 4: UAV_time = line.split(':')[1].split(' ')[1]
                if index == 5: WPT_consumption = line.split(':')[1].split(' ')[1]
        return UAV_consumption, UAV_time, WPT_consumption
    
    def getAllPathResultMovement(directory_path):
        all_solutions = []
        solutions = os.listdir(directory_path)

        # Define a function to extract the numeric part from the folder name
        def extract_number(folder_name):
            match = re.search(r'\d+', folder_name)
            return int(match.group()) if match else float('inf')

        # Sort the list using the extracted number
        solutions = sorted(solutions, key=extract_number)
        for solution in solutions:
            solution_dir = directory_path + solution + '/'
            UAV_consumption, UAV_time, WPT_consumption = readSolutionResults(solution_dir)
            solution_path = readPathFile(solution_dir)
            solution_movement = readMovementFile(solution_dir)
            # , solution_path, solution_movement, 
            all_solutions.append([solution, UAV_consumption, UAV_time, WPT_consumption])
        return all_solutions
    
    def processDifferentSolutionAmount(all_solutions1, all_solutions2):
        different_solutions = []
        solutions_amount1 = len(all_solutions1) 
        solutions_amount2 = len(all_solutions2)
        all_solutions1_tmp = all_solutions1.copy()
        all_solutions2_tmp = all_solutions2.copy()

        if solutions_amount1 > solutions_amount2: 
            for index, solution_nowind in enumerate(all_solutions2_tmp):
                solution_wind = all_solutions1_tmp[index]
                is_different = True
                while is_different == True:
                    if solution_nowind[0] != solution_wind[0]:
                            different_solutions.append(all_solutions1_tmp.pop(index))
                            solution_wind = all_solutions1_tmp[index]
                    else: is_different = False
            return 'first had more solutions', all_solutions1_tmp, all_solutions2_tmp, different_solutions
        elif solutions_amount1 < solutions_amount2: 
            for index, solution_wind in enumerate(all_solutions1_tmp):
                solution_nowind = all_solutions2_tmp[index]
                is_different = True
                while is_different == True:
                    if solution_wind[0] != solution_nowind[0]:
                            different_solutions.append(all_solutions2_tmp.pop(index))
                            solution_nowind = all_solutions2_tmp[index]
                    else: is_different = False
            return 'second had more solutions', all_solutions1_tmp, all_solutions2_tmp, different_solutions
        else:
            def iterateAllSolutions(all_solutions1_tmp, all_solutions2_tmp):
                result = []
                for index, solution_1 in enumerate(all_solutions1_tmp):
                    continue_search = True
                    iteration = 0
                    while iteration < len(all_solutions2_tmp):
                        solution_2 = all_solutions2_tmp[index + iteration]
                        if solution_2[0] == solution_1[0]:
                            break
                        else:
                            iteration += 1
                            if continue_search == True and (iteration + index) == len(all_solutions2_tmp):
                                iteration = 0
                                continue_search = False
                            if continue_search == False and (iteration + index) == len(all_solutions2_tmp):
                                result.append(all_solutions1_tmp.pop(index))
                                break
                return result
            different_solutions1 = iterateAllSolutions(all_solutions1_tmp, all_solutions2_tmp)
            different_solutions2 = iterateAllSolutions(all_solutions2_tmp, all_solutions1_tmp)
            different_1, different_2 = len(different_solutions1), len(different_solutions2)
            if different_1 == 0 and different_2 == 0: return 'solution amount the same', all_solutions1_tmp, all_solutions2_tmp, different_solutions
            elif different_1 != 0 and different_2 == 0: return 'first had differnet solutions', all_solutions1_tmp, all_solutions2_tmp, different_solutions1
            elif different_1 == 0 and different_2 != 0: return 'second had differnet solutions', all_solutions1_tmp, all_solutions2_tmp, different_solutions2
            else: return 'both had differnet solutions', all_solutions1_tmp, all_solutions2_tmp, [different_solutions1, different_solutions2]
    
    def seekDifferentPath(solutions_wind, solutions_nowind):
        different_paths = []
        reverse_paths = []
        same_paths = []
        for solution_wind in solutions_wind:
            solution_nowind = next((sublist for sublist in solutions_nowind if solution_wind[0] in sublist), None)
            # try: print(solution_nowind[0], solution_wind[0], test_directory)
            # except: print(solution_wind[0], test_directory)
            if solution_wind[3] != solution_nowind[3]:
                if solution_wind[3] == solution_nowind[3][::-1]:
                    reverse_paths.append(solution_wind[0])
                else:
                    # print(f'at {solution_wind[0]}')
                    # print(f'wind path {solution_wind[2]}')
                    # print(f'nowind path {solution_nowind[2]} \n')

                    different_paths.append(solution_wind[0])
            else: same_paths.append(solution_wind[0])
        return different_paths, reverse_paths, same_paths
    
    def getSinglePathResultMovement(directory_path, case):
        solution = []
        directory_path = directory_path + case + '/'
        UAV_consumption, UAV_time, WPT_consumption = readSolutionResults(directory_path)
        solution_path = readPathFile(directory_path)
        solution_movement = readMovementFile(directory_path)
        solution.extend([case, UAV_consumption, UAV_time, solution_path, solution_movement, WPT_consumption])
        return solution
        
    # This is just for flying, need to add hovering as well
    def getPathConsumptionInMovement(path, movement):
        last_destination = 0
        actual_path_cost, total_path_cost = 0, 0
        for index, destination in enumerate(path):
            if index > 0: 
                actual_path_cost = float(movement[last_destination][destination])
                total_path_cost += actual_path_cost
            last_destination = destination
        return total_path_cost
    
    def getAverageConsumption(all_solutions):
        consumption = [0,0,0]
        num_solutions = len(all_solutions)
        for solution in all_solutions:
            consumption[0] += float(solution[1][0]) #Total
            consumption[1] += float(solution[1][1]) #Hovering
            consumption[2] += float(solution[1][2]) #Flying
        
        return [(consumption[0]/num_solutions), \
                (consumption[1]/num_solutions), \
                (consumption[2]/num_solutions)]
    
    all_solutions_wind, all_solutions_nowind, all_solutions_nowind_inwind = [], [], []
    for alg_name in alg_name_list:
        working_directory_path = csv_path + test_directory
        working_directory_path_wind = working_directory_path + alg_name + '_wind/'
        working_directory_path_nowind = working_directory_path + alg_name + '_nowind/'
        working_directory_path_nowind_inwind = working_directory_path + alg_name + '_nowind-inwind/'
        all_solutions_wind.append(getAllPathResultMovement(working_directory_path_wind))
        all_solutions_nowind.append(getAllPathResultMovement(working_directory_path_nowind))
        all_solutions_nowind_inwind.append(getAllPathResultMovement(working_directory_path_nowind_inwind))

    return [[all_solutions_wind], [all_solutions_nowind], [all_solutions_nowind_inwind]]

    consumption_wind = getAverageConsumption(all_solutions_wind)
    consumption_nowind = getAverageConsumption(all_solutions_nowind)
    consumption_nowind_inwind = getAverageConsumption(all_solutions_nowind_inwind)

    # print(test_directory, f'{consumption_wind[0]:.2f}/{consumption_wind[1]:.2f}/{consumption_wind[2]:.2f}, {consumption_nowind[0]:.2f}/{consumption_nowind[1]:.2f}/{consumption_nowind[2]:.2f}')

    process_result, all_solutions_wind_1, all_solutions_nowind_1, different_solutions_wnw = processDifferentSolutionAmount(all_solutions_wind, all_solutions_nowind)
    # print('WIND NOWIND', test_directory, process_result, list(map(lambda x: x[0], different_solutions_wnw)))
    process_result, all_solutions_wind_2, all_solutions_nowind_2, different_solutions_nwinw = processDifferentSolutionAmount(all_solutions_nowind_inwind, all_solutions_nowind)
    # print('NOWIND-INWIND NOWIND', test_directory, process_result, list(map(lambda x: x[0], different_solutions_nwinw)))
    process_result, all_solutions_wind_3, all_solutions_nowind_3, different_solutions_wnwi = processDifferentSolutionAmount(all_solutions_wind, all_solutions_nowind_inwind)
    # print('WIND NOWIND-INWIND', test_directory, process_result, list(map(lambda x: x[0], different_solutions_wnwi)))
    # print('\n')    

def compareDifferentSolutions(data):
    def process_data(data):
        terrain_results = {}
        terrain_data = {}
        for item in data:
            comparison, terrain, solutions = item
            if terrain not in terrain_data:
                terrain_data[terrain] = {'WIND': 0, 'NOWIND': 0, 'INWIND': 0}
            
            if solutions:
                if comparison == 'WIND:NOWIND':
                    terrain_data[terrain]['WIND'] += len(solutions)
                elif comparison == 'NOWIND:INWIND':
                    terrain_data[terrain]['NOWIND'] += len(solutions)
                elif comparison == 'WIND:INWIND':
                    terrain_data[terrain]['INWIND'] += len(solutions)
        
        for terrain in terrain_data:
            if terrain_data[terrain]['WIND'] and terrain_data[terrain]['INWIND']: terrain_results[terrain] = 'WIND MORE'
            elif terrain_data[terrain]['WIND'] and terrain_data[terrain]['NOWIND']: terrain_results[terrain] = 'WIND:INWIND SAME'
            else: terrain_results[terrain] = 'ALL SAME'

        print(terrain_results)
        return terrain_results
    
    terrain_results = process_data(data)
    data = terrain_results

    # Restructure the data
    locations = ['TongMen', 'LiYu']
    degrees = [150, 250]
    kts_values = [7]

    # Create a DataFrame to hold the restructured data
    df = pd.DataFrame(index=pd.MultiIndex.from_product([locations, kts_values], names=['Location', 'kts']),
                    columns=degrees)

    # Fill the DataFrame
    for key, value in data.items():
        location = next(loc for loc in locations if loc in key)
        deg = int(key.split('deg')[0].split('km')[1])
        kts = int(key.split('kts')[0].split('deg')[1])
        df.loc[(location, kts), deg] = value

    # Create a mapping of unique values to numerical values for the heatmap
    unique_values = np.unique(df.values)
    value_map = {val: i for i, val in enumerate(unique_values)}
    df_numeric = df.applymap(value_map.get)

    # Set up the plot
    plt.figure(figsize=(12, 8))

    # Create the heatmap
    sns.heatmap(df_numeric, annot=df, fmt='', cmap='YlOrRd', linewidths=0.5, cbar=False)

    # Adjust the plot
    plt.xlabel('Degrees')
    plt.ylabel('Location and speed (kts)')

    # Show the plot
    plt.tight_layout()
    plt.show()

def compareIncreasingSN(data):

    # Creating the plot
    plt.figure(figsize=(18, 10))

    for alg_num, alg in enumerate(alg_name_list):
        if alg != 'ACO': continue
        for wtype in range(3):
            type = 'NA'
            if wtype == 0: 
                type = 'Wind'
            if wtype == 1: 
                type = 'Nowind'
                # continue
            if wtype == 2: 
                type = 'Nowind_Inwind'
                # continue

            # Extracting values
            test_cases = [int(d[0][2:]) for d in data[alg_num][wtype]]
            total_power = [float(d[1][0]) for d in data[alg_num][wtype]]
            hover_power = [float(d[1][1]) for d in data[alg_num][wtype]]
            flight_power = [float(d[1][2]) for d in data[alg_num][wtype]]
            time_taken = [float(d[2]) for d in data[alg_num][wtype]]
            charging_power = [float(d[3]) for d in data[alg_num][wtype]]

            plt.plot(test_cases, total_power, marker='o', label=(alg + ' Total Power ' + type))
            # plt.plot(test_cases, hover_power, marker='o', label=(alg + ' Hover Power ' + type))
            # plt.plot(test_cases, flight_power, marker='o', label=(alg + ' Flight Power ' + type))
            # plt.plot(test_cases, time_taken, marker='o', label=(alg + ' Time Taken ' + type))
            plt.plot(test_cases, charging_power, marker='o', label=(alg + ' Charging Power ' + type))

    plt.xlabel('Sensors')
    plt.ylabel('Charge Used (mAh)')
    plt.title('UAV Performance Metrics Uniform Sensor Generation')
    plt.legend()
    plt.grid(True)

    plt.show()

all_results = []
for iteration in range(1):
    if iteration == 0: wind_name_extention_list = wind_name_extention_list_7kts
    # elif iteration == 1: wind_name_extention_list = wind_name_extention_list_7kts

    for test_name in test_name_list:
        for wind_name_extention in wind_name_extention_list: 
            results = main(test_name, wind_name_extention)
            results = [item for sublist in results for item in sublist]
            all_results.append(results)


def averageAllSolutions(input_list):
    def removeOutlierSolutions(input_list):
        def getOutliersIndex(data):
            # Convert to numpy array
            data = np.array(data)

            # Calculate Q1 and Q3
            Q1 = np.percentile(data, 25)
            Q3 = np.percentile(data, 75)

            # Calculate IQR
            IQR = Q3 - Q1

            # Determine outlier bounds
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR

            # Identify outliers
            outliers = [(i, val) for i, val in enumerate(data) if val < lower_bound or val > upper_bound]

            # Output indices of outliers
            outlier_indices = [i for i, _ in outliers]
            return outlier_indices
        
        def removeOutliers(index_list, data_list):
            # Sort the index list in descending order to avoid affecting the positions of unprocessed elements
            sorted_indices = sorted(index_list, reverse=True)
            
            # Remove elements from data_list based on the sorted indices
            for index in sorted_indices:
                if 0 <= index < len(data_list):
                    del data_list[index]
            
            return data_list
        total_power = removeOutliers(outliers_index, total_power)
        
        return 

    # Helper function to get empty list from input
    def makeEmptyListFromShape(lst):
        if isinstance(lst, list):
            return [makeEmptyListFromShape(lst[0]) for _ in range(len(lst))]
        return 0
    
    # Initialize a 2D list to store the sum of each position
    sums = makeEmptyListFromShape(input_list)
    sums = sums[0]

    # Sum the corresponding elements from each inner list across all outer lists
    for case, outer_list in enumerate(input_list): # Iter: test case
        for alg in range(len(input_list[0])): # Iter: ACO, GA, BH
            for wtype in range(len(input_list[0][0])): # Iter: wind, nowind, nowind_inwind
                for sn in range(len(input_list[0][0][0])): # Iter: SN number
                    for i, val in enumerate(outer_list[alg][wtype][sn]): # Iter: value
                        if 'SN' in val: sums[alg][wtype][sn][i] = val # Handle Name
                        elif isinstance(val, list):  # Handle sublists
                            for sublist_i, sublist_item in enumerate(val):   
                                if case == 0:
                                    if isinstance(sums[alg][wtype][sn][i], int): sums[alg][wtype][sn][i] = []
                                    sums[alg][wtype][sn][i].append(float(sublist_item))
                                else: sums[alg][wtype][sn][i][sublist_i] += float(sublist_item)
                        else: sums[alg][wtype][sn][i] += float(val)
    
    # Calculate the average for each position
    def getAverage(list, value):
        return [
                    [
                        [
                            [
                                item[0],  # Keep the string value as is
                                [x / value for x in item[1]],  # Divide each element in the inner list by 3
                                item[2] / value,  # Divide the scalar value by 3
                                item[3] / value,   # Divide the scalar value by 3
                            ]
                            for item in subsublist
                        ]
                        for subsublist in sublist
                    ]
                    for sublist in list
                ]
    averages = getAverage(sums, len(input_list))
    return averages

# Example usage
three_d_named_lists = [
        [   [[['SN1', ['311.98', '224.44', '87.53'], '0.72', '3549.24']],
             [['SN2', ['310.50', '225.30', '88.10'], '0.75', '3550.50']],
             [['SN1', ['312.00', '223.90', '87.90'], '0.70', '3548.00']]],
        
            [[['SN1', ['311.00', '223.00', '86.00'], '0.71', '3547.00']],
             [['SN2', ['310.00', '224.00', '87.00'], '0.73', '3546.00']],
             [['SN1', ['312.00', '225.00', '88.00'], '0.74', '3549.00']]],
        
            [[['SN1', ['312.50', '225.50', '88.50'], '0.76', '3551.00']],
             [['SN2', ['311.50', '224.50', '87.50'], '0.72', '3548.50']],
             [['SN1', ['310.50', '223.50', '86.50'], '0.70', '3547.50']]]],
     
        [   [[['SN1', ['211.98', '124.44', '57.53'], '0.52', '2549.24']],
             [['SN2', ['210.50', '125.30', '58.10'], '0.55', '2550.50']],
             [['SN1', ['212.00', '123.90', '57.90'], '0.50', '2548.00']]],
     
            [[['SN1', ['211.00', '123.00', '56.00'], '0.51', '2547.00']],
             [['SN2', ['210.00', '124.00', '57.00'], '0.53', '2546.00']],
             [['SN1', ['212.00', '125.00', '58.00'], '0.54', '2549.00']]],
     
            [[['SN1', ['212.50', '125.50', '58.50'], '0.56', '2551.00']],
             [['SN2', ['211.50', '124.50', '57.50'], '0.52', '2548.50']],
             [['SN1', ['210.50', '123.50', '56.50'], '0.50', '2547.50']]]],
     
        [   [[['SN1', ['111.98', '24.44', '27.53'], '0.32', '1549.24']],
             [['SN2', ['110.50', '25.30', '28.10'], '0.35', '1550.50']],
             [['SN1', ['112.00', '23.90', '27.90'], '0.30', '1548.00']]],
     
            [[['SN1', ['111.00', '23.00', '26.00'], '0.31', '1547.00']],
             [['SN2', ['110.00', '24.00', '27.00'], '0.33', '1546.00']],
             [['SN1', ['112.00', '25.00', '28.00'], '0.34', '1549.00']]],
     
            [[['SN1', ['112.50', '25.50', '28.50'], '0.36', '1551.00']],
             [['SN2', ['111.50', '24.50', '27.50'], '0.32', '1548.50']],
             [['SN1', ['110.50', '23.50', '26.50'], '0.30', '1547.50']]]]
]
shape = lambda x: (len(x), *shape(x[0])) if isinstance(x, list) else ()
average_results = averageAllSolutions(all_results)

def printAll(sn):
    def printAllSolutions(sn):
        for i, results in enumerate(all_results):
            print(i + 1)
            print(results[0][sn])
            print(results[1][sn])
            print(results[2][sn])
    def printAllAverage(sn): 
        print('average')
        for i in range(3): print(average_results[i][sn])
    printAllSolutions(sn)
    printAllAverage(sn)
# printAll(20)
# compareDifferentSolutions(different_solutions)
compareIncreasingSN(average_results)