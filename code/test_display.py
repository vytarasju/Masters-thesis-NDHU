from read_write_data import *
from plot import *

# This is for wind visualisation only
# !!!Test directory should be in code_path/csv!!!
"""BEGIN Directory Initialisation"""
csv_path = '/home/vytska/thesis/code/csv/'
test_directory = 'output_NASADEM_old'
test_case = 'D3.0SP3'

working_directory_path = csv_path + test_directory + '/'
readCSV_path = working_directory_path.replace(csv_path, '')
working_directory_path += test_case + '/'
"""END Directory Initialisation"""

"""BEGIN Data Read"""
terrain = readCSV(readCSV_path + 'terrain_data.csv')
wind = readCSV(readCSV_path + 'wind_data.csv')
readCSV_path += test_case + '/'
sensors = readCSV(readCSV_path + 'sensors.csv')
centroids = readCSV(readCSV_path + 'clusters.csv')

path = []
with open(working_directory_path + 'path.txt', 'r') as file:
    for line in file:
        line = line.split(', ')
        trans_table = str.maketrans('', '', '[]')
        line = [int(item.translate(trans_table)) for item in line]
        path = line

UAV_consumption = 0
UAV_time = 0
with open(working_directory_path + 'solution_results.txt', 'r') as file:
    for index, line in enumerate(file):
        if index == 1: UAV_consumption = line.split(':')[1].split(' ')[1]
        if index == 2: UAV_time = line.split(':')[1].split(' ')[1]

motion = []
with open(working_directory_path + 'motion.csv', 'r') as file:
    reader = csv.reader(file)
    current_motion = []
    motion_to, motion_from = 0,0
    for index, line in enumerate(reader):
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
"""END Data Read"""

plot(terrain=terrain, centroids=centroids, sensors=sensors, path=path, motion=motion)
