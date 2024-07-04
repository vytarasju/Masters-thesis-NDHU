from read_write_data import *
from plot import *

# This is for wind nowind test visualisation and comparison only
# !!!Test directory should be in code_path/csv!!!
"""BEGIN Directory Initialisation"""
csv_path = '/home/vytska/thesis/code/csv/'
test_directory = 'output_NASADEM_old_combined'
test_case = 'D0.5SP21'

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
    UAV_consumption = 0
    UAV_time = 0
    with open(directory_path + 'solution_results.txt', 'r') as file:
        for index, line in enumerate(file):
            if index == 1: UAV_consumption = line.split(':')[1].split(' ')[1]
            if index == 2: UAV_time = line.split(':')[1].split(' ')[1]
    return UAV_consumption, UAV_time
UAV_consumption_wind, UAV_time_wind = readSolutionResults(working_directory_path_wind)
UAV_consumption_nowind, UAV_time_nowind = readSolutionResults(working_directory_path_nowind)

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
def getAllPathAndResult(directory_path):
    all_solutions = []
    directory_path = directory_path[:-(len(test_case) + 1)]
    solutions = os.listdir(directory_path)
    for solution in solutions:
        solution_path = directory_path + solution + '/'
        UAV_consumption, UAV_time = readSolutionResults(solution_path)
        solution_path = readPathFile(solution_path)
        all_solutions.append([solution, UAV_consumption, solution_path])
    return all_solutions

all_solutions_wind = getAllPathAndResult(working_directory_path_wind)
all_solutions_nowind = getAllPathAndResult(working_directory_path_nowind)

def seekDifferentPath():
    different_paths = []
    reverse_paths = []
    for solution_wind in all_solutions_wind:
        solution_nowind = next((sublist for sublist in all_solutions_nowind if solution_wind[0] in sublist), None)
        if solution_wind[2] != solution_nowind[2]:
            if solution_wind[2] == solution_nowind[2][::-1]:
                reverse_paths.append(solution_wind[0])
            else:
                print(f'at {solution_wind[0]}')
                print(f'wind path {solution_wind[2]}')
                print(f'nowind path {solution_nowind[2]} \n')

                different_paths.append(solution_wind[0])

# This is just for flying, need to add hovering as well
def compareNowindActualConsumption():
    last_destination = 0
    actual_path_cost = 0
    for index, destination in enumerate(path_nowind):
        if index == 0: 
            last_destination = destination
            continue
        actual_path_cost += float(movement_wind[last_destination][destination])
    return actual_path_cost
actual_nowind_cost = compareNowindActualConsumption()
"""END Processing Data"""

"""BEGIN Visualisation Functions"""
def plotBoth():
    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    plot(terrain, centroids_wind, sensors_wind, motion_wind, path_wind, subplot_ax=ax1, ax_title='UAV Path Planning with Wind Simulation')

    ax2 = fig.add_subplot(122, projection='3d')
    plot(terrain, centroids_nowind, sensors_nowind, motion_nowind, path_nowind, subplot_ax=ax2, ax_title='UAV Path Planning without Wind Simulation')

    plt.show()



print(f'WIND Battery Consumption {UAV_consumption_wind} mAh')
print(f'and its path {path_wind} \n')

print(f'NO WIND Battery Consumption {UAV_consumption_nowind} mAh')
print(f'Actual NO WIND Battery Consumption would be {actual_nowind_cost} mAh')
print(f'and its path {path_nowind}')
compareNowindActualConsumption()
plotBoth()
"""END Visualisation Functions"""
