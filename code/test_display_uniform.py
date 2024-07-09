from read_write_data import *
from plot import *

# This is for wind nowind test visualisation and comparison only
# !!!Test directory should be in code_path/csv!!!
"""BEGIN Directory Initialisation"""
csv_path = '/home/vytska/thesis/code/csv/'
test_directory = 'LiYu1km_combined_uniform_150deg10kts'
test_case = 'SN30'

working_directory_path = csv_path + test_directory + '/'
readCSV_path = working_directory_path.replace(csv_path, '')
working_directory_path_wind = working_directory_path + 'wind/' + test_case + '/'
working_directory_path_nowind = working_directory_path + 'nowind/' + test_case + '/'
"""END Directory Initialisation"""

"""BEGIN Data Read"""
terrain = readCSV(readCSV_path + 'terrain_data.csv')
wind = readCSV(readCSV_path + 'wind_data.csv')
readCSV_path_wind = readCSV_path + 'wind/' + test_case + '/'
readCSV_path_nowind = readCSV_path + 'nowind/' + test_case + '/'

sensors_wind = readCSV(readCSV_path_wind + 'sensors.csv')
centroids_wind = readCSV(readCSV_path_wind + 'clusters.csv')
sensors_nowind = readCSV(readCSV_path_wind + 'sensors.csv')
centroids_nowind = readCSV(readCSV_path_wind + 'clusters.csv')

def readMovementFile(directory_path):
    movement = []
    with open(directory_path + 'movement.csv', 'r') as file:
        reader = csv.reader(file)
        for line in reader: movement.append(line)
        return movement
movement_wind = readMovementFile(working_directory_path_wind)
movement_nowind = readMovementFile(working_directory_path_nowind)

# def readMovementWindFile(directory_path):
#     movement = []
#     with open(directory_path + 'movement_wind.csv', 'r') as file:
#         reader = csv.reader(file)
#         for line in reader: movement.append(line)
#         return movement
# movement_wind_nowind = readMovementWindFile(working_directory_path_nowind)

def readPathFile(directory_path):
    with open(directory_path + 'path.txt', 'r') as file:
        for line in file:
            line = line.split(', ')
            trans_table = str.maketrans('', '', '[]')
            line = [int(item.translate(trans_table)) for item in line]
            path = line
    return path
path_wind = readPathFile(working_directory_path_wind)
path_nowind = readPathFile(working_directory_path_nowind)

def readSolutionResults(directory_path):
    UAV_consumption = []
    UAV_time = 0
    with open(directory_path + 'solution_results.txt', 'r') as file:
        for index, line in enumerate(file):
            if index > 0 and index < 4: UAV_consumption.append(line.split(':')[1].split(' ')[1])
            if index == 4: UAV_time = line.split(':')[1].split(' ')[1]
            if index == 5: WPT_consumption = line.split(':')[1].split(' ')[1]
    return UAV_consumption, UAV_time, WPT_consumption
UAV_consumption_wind, UAV_time_wind, WPT_consumption_wind = readSolutionResults(working_directory_path_wind)
UAV_consumption_nowind, UAV_time_nowind, WPT_consumption_nowind = readSolutionResults(working_directory_path_nowind)

def readMotion(directory_path):
    motion = []
    with open(directory_path + 'motion.csv', 'r') as file:
        reader = csv.reader(file)
        current_motion = []
        motion_to, motion_from = 0,0
        for line in reader:
            # Convert motion saved as a string to a correct format
            current_motion = line[0].split(', ')
            trans_table = str.maketrans('', '', '[]()')
            current_motion = [item.translate(trans_table) for item in current_motion]

            point = []
            points = []
            coordinate_counter = 0
            for item in current_motion:
                point.append(float(item))
                coordinate_counter += 1
                if coordinate_counter == 3:
                    points.append(point)
                    coordinate_counter = 0
                    point = []
            current_motion = points
            # Convert from and to cluster points to integers
            motion_to = int(line[1])
            motion_from = int(line[2])

            # Append all in a specific format
            motion.append([current_motion, motion_to, motion_from])
    return motion
motion_wind = readMotion(working_directory_path_wind)
motion_nowind = readMotion(working_directory_path_nowind)
"""END Data Read"""

"""BEGIN Processing Data"""
def getSinglePathResultMovement(directory_path, case):
    solution = []
    directory_path = directory_path[:-(len(test_case) + 1)] + case + '/'
    UAV_consumption, UAV_time, WPT_consumption = readSolutionResults(directory_path)
    solution_path = readPathFile(directory_path)
    solution_movement = readMovementFile(directory_path)
    solution.extend([case, UAV_consumption, UAV_time, solution_path, solution_movement, WPT_consumption])
    return solution

def getAllPathResultMovement(directory_path):
    all_solutions = []
    directory_path = directory_path[:-(len(test_case) + 1)]
    solutions = os.listdir(directory_path)
    for solution in solutions:
        solution_dir = directory_path + solution + '/'
        UAV_consumption, UAV_time, WPT_consumption = readSolutionResults(solution_dir)
        solution_path = readPathFile(solution_dir)
        solution_movement = readMovementFile(solution_dir)
        all_solutions.append([solution, UAV_consumption, UAV_time, solution_path, solution_movement, WPT_consumption])
    return all_solutions
all_solutions_wind = getAllPathResultMovement(working_directory_path_wind)
all_solutions_nowind = getAllPathResultMovement(working_directory_path_nowind)

    
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
    else: return 'solution amount the same', all_solutions_wind, all_solutions_nowind, different_solutions
process_result, all_solutions_wind, all_solutions_nowind, different_solutions = processDifferentSolutionAmount(all_solutions_wind, all_solutions_nowind)
print(process_result)

def seekDifferentPath():
    different_paths = []
    reverse_paths = []
    same_paths = []
    for solution_wind in all_solutions_wind:
        solution_nowind = next((sublist for sublist in all_solutions_nowind if solution_wind[0] in sublist), None)
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
actual_nowind_cost = getPathConsumptionInMovement(path_nowind, movement_wind)

def getAverageConsumption():
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
    directory_path_nowind = working_directory_path_nowind[:-(len(test_case) + 1)]

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

def compareValuesProc(a, b):
    if a == b:
        return "The values are equal."
    elif a > b:
        difference = ((a - b) / b) * 100
        return f"The first value is bigger by {difference:.2f}%"
    else:
        difference = ((b - a) / a) * 100
        return f"The second value is bigger by {difference:.2f}%"

"""END Processing Data"""

"""BEGIN Visualisation Functions"""
def plotBoth():
    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    plot(terrain, centroids_wind, sensors_wind, motion_wind, path_wind, subplot_ax=ax1, ax_title='UAV Path Planning with Wind Simulation')

    ax2 = fig.add_subplot(122, projection='3d')
    plot(terrain, centroids_nowind, sensors_nowind, motion_nowind, path_nowind, subplot_ax=ax2, ax_title='UAV Path Planning without Wind Simulation')

    plt.show()

"""Check for not the same amount of solutions"""

"""GET consumtion differences"""
# fly_wind, fly_nowind, fly_nowind_againstwind, hover_wind, hover_nowind, \
#     total_wind, total_nowind, WPT_wind, WPT_nowind = getAverageConsumption()
# print(f'Fly consumption: ')
# print(f'Wind: {fly_wind:.2f}, Nowind: {fly_nowind:.2f}')
# print(f'Nowind real: {fly_nowind_againstwind:.2f}')

# print(f'\nHover consumption:')
# print(f'Wind: {hover_wind:.2f}, Nowind: {hover_nowind:.2f}')

# print(f'\nTotal consumption:')
# print(f'Wind: {total_wind:.2f}, Nowind: {total_nowind:.2f}')

# print(f'\nWPT consumption:')
# print(f'Wind: {(WPT_wind / 1000):.2f}, Nowind: {(WPT_nowind / 1000):.2f}')

# print(f'first total wind, second nowind: {compareValuesProc(total_wind, total_nowind)}')


"""GET different paths"""
# different_paths, reverse_pathsm, same_paths = seekDifferentPath()
# print(different_paths)

"""GET different paths with wind initial lower cost"""
# directory_path_nowind = working_directory_path_nowind[:-(len(test_case) + 1)]
# directory_path_wind = working_directory_path_wind[:-(len(test_case) + 1)]

# for index, solution in enumerate(different_paths):
#     solution_nowind = getSinglePathResultMovement(working_directory_path_nowind, solution)
#     solution_wind = getSinglePathResultMovement(working_directory_path_wind, solution)

#     consumption_fly_wind = float(solution_wind[1][2])
#     consumption_fly_nowind = float(solution_nowind[1][2])
#     if consumption_fly_wind < consumption_fly_nowind: print(solution)


"""GET specific path comparison"""
print(f'\n\nWIND Battery Consumption {UAV_consumption_wind[0]} mAh')
print(f'Hovering: {UAV_consumption_wind[1]} mAh')
print(f'Flying: {UAV_consumption_wind[2]} mAh')
print(f'Wind solution path sequence: {path_wind} \n')

print(f'NO WIND Battery Consumption {UAV_consumption_nowind[0]} mAh')
print(f'Hovering: {UAV_consumption_nowind[1]} mAh')
print(f'Flying: {UAV_consumption_nowind[2]} mAh')
print(f'No wind flight consumption in wind cost: {actual_nowind_cost:.2f} mAh')
print(f'No wind solution path sequence: {path_nowind}')
plotBoth()
"""END Visualisation Functions"""
