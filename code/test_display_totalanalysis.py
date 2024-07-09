from read_write_data import *
from plot import *
import matplotlib.pyplot as plt

test_name_list = ['TongMen1km', 'LiYu1km', 'YanZiKou1km', 'FuXing1km', 'ZiYouLi1km', 'DongHwa1km']
wind_name_extention_list_3kts = ['60deg3kts', '150deg3kts', '250deg3kts']
wind_name_extention_list_7kts = ['60deg7kts', '150deg7kts', '250deg7kts']
csv_path = '/home/vytska/thesis/code/csv/'

def main(test_name, wind_name_extention):
    test_directory = test_name + '_combined_centralized_' + wind_name_extention + '/'
    readCSV_path = test_directory
    working_directory_path = csv_path + test_directory + '/'
    working_directory_path_wind = working_directory_path + 'wind/'
    working_directory_path_nowind = working_directory_path + 'nowind/'
    
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
        for solution in solutions:
            solution_dir = directory_path + solution + '/'
            UAV_consumption, UAV_time, WPT_consumption = readSolutionResults(solution_dir)
            solution_path = readPathFile(solution_dir)
            solution_movement = readMovementFile(solution_dir)
            all_solutions.append([solution, UAV_consumption, UAV_time, solution_path, solution_movement, WPT_consumption])
        return all_solutions

    def processDifferentSolutionAmount(all_solutions_wind, all_solutions_nowind):
        different_solutions = []
        solutions_amount_wind = len(all_solutions_wind) 
        solutions_amount_nowind = len(all_solutions_nowind)
        if solutions_amount_wind > solutions_amount_nowind: 
            for index, solution_nowind in enumerate(all_solutions_nowind):
                solution_wind = all_solutions_wind[index]
                is_different = True
                while is_different == True:
                    if solution_nowind[0] != solution_wind[0]:
                            different_solutions.append(all_solutions_wind.pop(index))
                            solution_wind = all_solutions_wind[index]
                    else: is_different = False
            return 'wind had more solutions', all_solutions_wind, all_solutions_nowind, different_solutions
        elif solutions_amount_wind < solutions_amount_nowind: 
            for index, solution_wind in enumerate(all_solutions_wind):
                solution_nowind = all_solutions_nowind[index]
                is_different = True
                while is_different == True:
                    if solution_wind[0] != solution_nowind[0]:
                            different_solutions.append(all_solutions_nowind.pop(index))
                            solution_nowind = all_solutions_nowind[index]
                    else: is_different = False
            return 'nowind had more solutions', all_solutions_wind, all_solutions_nowind, different_solutions
        else:
            def iterateAllSolutions(all_solutions1, all_solutions2):
                result = []
                for index, solution_wind in enumerate(all_solutions1):
                    continue_search = True
                    iteration = 0
                    while iteration < len(all_solutions2):
                        solution_nowind = all_solutions2[index + iteration]
                        if solution_nowind[0] == solution_wind[0]:
                            break
                        else:
                            iteration += 1
                            if continue_search == True and (iteration + index) == len(all_solutions2):
                                iteration = 0
                                continue_search = False
                            if continue_search == False and (iteration + index) == len(all_solutions2):
                                result.append(all_solutions1.pop(index))
                                break
                return result
            different_solutions_wind = iterateAllSolutions(all_solutions_wind, all_solutions_nowind)
            different_solutions_nowind = iterateAllSolutions(all_solutions_nowind, all_solutions_wind)
            different_wind, different_nowind = len(different_solutions_wind), len(different_solutions_nowind)
            if different_wind == 0 and different_nowind == 0: return 'solution amount the same', all_solutions_wind, all_solutions_nowind, different_solutions
            elif different_wind != 0 and different_nowind == 0: return 'wind had differnet solutions', all_solutions_wind, all_solutions_nowind, different_solutions_wind
            elif different_wind == 0 and different_nowind != 0: return 'nowind had differnet solutions', all_solutions_wind, all_solutions_nowind, different_solutions_nowind
            else: return 'both had differnet solutions', all_solutions_wind, all_solutions_nowind, [different_solutions_wind, different_solutions_nowind]


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
    
    def readMovementWindFile(directory_path):
        movement = []
        with open(directory_path + 'movement_wind.csv', 'r') as file:
            reader = csv.reader(file)
            for line in reader: movement.append(line)
            return movement
        
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

    def getAverageConsumption(directory_path_nowind, all_solutions_wind):
        fly_wind = 0
        fly_nowind = 0
        fly_nowind_againstwind = 0

        hover_wind = 0
        hover_nowind = 0

        total_wind = 0
        total_nowind = 0

        WPT_wind = 0
        WPT_nowind = 0

        len_solution = len(all_solutions_wind)

        for solution_wind in all_solutions_wind:
            solution_dir_nowind = directory_path_nowind + solution_wind[0] + '/'
            solution_nowind = getSinglePathResultMovement(working_directory_path_nowind, solution_wind[0])

            fly_wind += float(solution_wind[1][2])
            fly_nowind += float(solution_nowind[1][2])

            movement_wind_nowind = readMovementWindFile(solution_dir_nowind)
            fly_nowind_againstwind += float(getPathConsumptionInMovement(solution_nowind[3], movement_wind_nowind))

            hover_wind += float(solution_wind[1][1])
            hover_nowind += float(solution_nowind[1][1])

            total_wind += float(solution_wind[1][0])
            total_nowind += float(solution_nowind[1][0])

            WPT_wind += float(solution_wind[5])
            WPT_nowind += float(solution_nowind[5])

        
        fly_wind /= len_solution
        fly_nowind /= len_solution
        hover_wind /= len_solution
        hover_nowind /= len_solution
        total_wind /= len_solution
        total_nowind /= len_solution
        WPT_wind /= len_solution
        WPT_nowind /= len_solution
        fly_nowind_againstwind /= len_solution
        return fly_wind, fly_nowind, fly_nowind_againstwind, hover_wind, hover_nowind, \
            total_wind, total_nowind, WPT_wind, WPT_nowind

    """Callout section"""
    all_solutions_wind = getAllPathResultMovement(working_directory_path_wind)
    all_solutions_nowind = getAllPathResultMovement(working_directory_path_nowind)
    process_result, all_solutions_wind, all_solutions_nowind, different_solutions = processDifferentSolutionAmount(all_solutions_wind, all_solutions_nowind)
    # print(test_directory, process_result)
    different_paths, reverse_paths, same_paths = seekDifferentPath(all_solutions_wind, all_solutions_nowind)

    fly_wind, fly_nowind, fly_nowind_againstwind, hover_wind, hover_nowind, \
    total_wind, total_nowind, WPT_wind, WPT_nowind = getAverageConsumption(working_directory_path_nowind, all_solutions_wind)
    # print(f'Fly consumption: ')
    # print(f'Wind: {fly_wind:.2f}, Nowind: {fly_nowind:.2f}')
    # print(f'Nowind real: {fly_nowind_againstwind:.2f}')

    # print(f'\nHover consumption:')
    # print(f'Wind: {hover_wind:.2f}, Nowind: {hover_nowind:.2f}')

    # print(f'\nTotal consumption:')
    # print(f'Wind: {total_wind:.2f}, Nowind: {total_nowind:.2f}')

    # print(f'\nWPT consumption:')
    # print(f'Wind: {(WPT_wind / 1000):.2f}, Nowind: {(WPT_nowind / 1000):.2f}')

    results = []
    results.append(len(all_solutions_wind)) #amount of paths
    results.append(len(different_paths)) #amount of unique paths
    results.append(len(reverse_paths)) #amount of reverse paths
    results.append(len(same_paths)) #amount of same paths
    results.append([total_wind, fly_wind, hover_wind]) #consumptin wind
    results.append([total_nowind, fly_nowind, hover_nowind]) #consumptin wind

    return results

def graphPlotPath(path_slow, path_fast):
    # Extract data for plotting
    names = [item[0] for item in path_slow]
    speed3_dif_path = [item[2] for item in path_slow]
    speed3_reverse_path = [item[3] for item in path_slow]
    speed3_same_path = [item[4] for item in path_slow]

    speed7_dif_path = [item[2] for item in path_fast]
    speed7_reverse_path = [item[3] for item in path_fast]
    speed7_same_path = [item[4] for item in path_fast]


    # Set up the figure and axis
    fig, ax = plt.subplots()

    # Define the positions for the groups
    bar_width = 0.1
    gap = 0.1
    index = range(len(names))

    
    # Plot the bars with added gap
    bar1 = ax.barh(index, speed3_dif_path, height=bar_width, label='3kts Different Paths')
    bar2 = ax.barh([i + bar_width for i in index], speed3_reverse_path, height=bar_width, label='3kts Reverse Paths')
    bar3 = ax.barh([i + 2 * bar_width for i in index], speed3_same_path, height=bar_width, label='3kts Same Paths')
    bar4 = ax.barh([i + 3 * bar_width + gap for i in index], speed7_dif_path, height=bar_width, label='7kts Different Paths')
    bar5 = ax.barh([i + 4 * bar_width + gap for i in index], speed7_reverse_path, height=bar_width, label='7kts Reverse Paths')
    bar6 = ax.barh([i + 5 * bar_width + gap for i in index], speed7_same_path, height=bar_width, label='7kts Same Paths')

    # Add labels and title
    ax.set_ylabel('Locations')
    ax.set_xlabel('Percentage')
    ax.set_title('Path Distribution by Location')
    ax.set_yticks([i + 2.5 * bar_width + gap / 2 for i in index])
    ax.set_yticklabels(names)
    ax.legend()

    # Show the plot
    plt.show()

def graphPlotConsumption(wind_3kts, wind_7kts):
    # Prepare data for plotting
    locations = [data[0] for data in wind_7kts]
    x = np.arange(len(locations))

    # Set up the plot
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot data for 7kts wind
    ax.plot(x, [data[1] for data in wind_7kts], 'bo-', label='7kts - With Wind', alpha=0.7)
    ax.plot(x, [data[2] for data in wind_7kts], 'bx--', label='7kts - Without Wind', alpha=0.7)
    ax.scatter(x, [data[3] for data in wind_7kts], color='blue', s=80, marker='^', label='7kts - Difference', alpha=0.7)

    # Plot data for 3kts wind
    ax.plot(x, [data[1] for data in wind_3kts], 'ro-', label='3kts - With Wind', alpha=0.7)
    ax.plot(x, [data[2] for data in wind_3kts], 'rx--', label='3kts - Without Wind', alpha=0.7)
    ax.scatter(x, [data[3] for data in wind_3kts], color='red', s=80, marker='^', label='3kts - Difference', alpha=0.7)

    # Customize the plot
    ax.set_xlabel('Locations')
    ax.set_ylabel('Battery Consumption')
    ax.set_title('UAV Energy Consumption Comparison by Location')
    ax.set_xticks(x)
    ax.set_xticklabels(locations, rotation=25, ha='right')
    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))

    # Add a grid for better readability
    ax.grid(True, linestyle='--', alpha=0.7)

    # Adjust layout and display the plot
    plt.tight_layout()
    plt.show()

path_slow, path_fast = [], []
consumption_slow, consumption_fast = [], []
for iteration in range(2):
    paths = []
    consumption = []
    if iteration == 0: wind_name_extention_list = wind_name_extention_list_3kts
    elif iteration == 1: wind_name_extention_list = wind_name_extention_list_7kts

    for test_name in test_name_list:
        total_paths, different_paths = 0, 0
        reverse_paths, same_paths = 0, 0
        total_consumption_wind, total_consumption_nowind = 0, 0
        total_flight_wind, total_flight_nowind = 0, 0
        for wind_name_extention in wind_name_extention_list: 
            results = main(test_name, wind_name_extention)
            total_paths += results[0]
            different_paths += results[1]
            reverse_paths += results[2]
            same_paths += results[3]
            total_consumption_wind += results[4][0]
            total_consumption_nowind += results[5][0]
            total_flight_wind += results[4][1]
            total_flight_nowind += results[5][1]

        def getPercengate(value): return (value/total_paths) * 100
        def getConsumption(value): return (value/len(wind_name_extention_list))
        paths.append([test_name, total_paths, getPercengate(different_paths), getPercengate(reverse_paths), getPercengate(same_paths)])
        consumption.append([test_name, getConsumption(total_consumption_wind), getConsumption(total_consumption_nowind), getConsumption(total_consumption_nowind - total_consumption_wind)])
        # if iteration == 0: print(f'for {test_name} 3kts winds')
        # if iteration == 1: print(f'for {test_name} 7kts winds')

        # print(f'path amount is {total_paths}: unique {(getPercengate(different_paths)):.2f}%, reverse {getPercengate(reverse_paths):.2f}%, same {getPercengate(same_paths):.2f}%')
        # print(f'consumption wind {total_consumption_wind:.2f}, nowind {total_consumption_nowind:.2f}, difference {(total_consumption_nowind - total_consumption_wind):.2f}')
        # print('\n')
    
    if iteration == 0: 
        path_slow = paths
        consumption_slow = consumption
    elif iteration == 1: 
        path_fast = paths
        consumption_fast = consumption

# print(consumption_fast, '\n', consumption_slow)
graphPlotConsumption(consumption_slow, consumption_fast)
# graphPlotPath(path_slow, path_fast)