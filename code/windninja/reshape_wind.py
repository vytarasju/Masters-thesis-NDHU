import sys
path_windninja = "/home/vytska/thesis/code/windninja/"
path_terrain = path_windninja + "converted/"
path_wind = path_windninja + "wind/"


# Check if arguments are provided
if len(sys.argv) < 1:
    sys.exit("No input provided")

terrain_filename = sys.argv[1]
wind_vel_filename = sys.argv[2]
wind_ang_filename = sys.argv[3]

terrain_dir = path_terrain + terrain_filename
wind_vel_dir = path_wind + wind_vel_filename
wind_ang_dir = path_wind + wind_ang_filename

terrain_dimensions = []
with open(terrain_dir, 'r') as file:
    for index, line in enumerate(file):
        if index == 0: terrain_dimensions.append(['col', line.split()[1]])
        elif index == 1: terrain_dimensions.append(['row', line.split()[1]])
        else: break

def readWind(data_dir):
    data = []
    metadata = []
    with open(data_dir, 'r') as file:
        for index, line in enumerate(file):
            if index < 6:
                metadata.append(line)
            else: data.append(line.split())
    return metadata, data
wind_vel_metadata, wind_vel_data = readWind(wind_vel_dir)
wind_ang_metadata, wind_ang_data = readWind(wind_ang_dir)

def matchWindWithTerrain(terrain, wind_metadata, wind_data):
    wind_col = int(wind_metadata[0].split()[1])
    wind_row = int(wind_metadata[1].split()[1])

    terrain_col = int(terrain[0][1])
    terrain_row = int(terrain[1][1])

    wind_data_changed = []

    # Change data
    if wind_col > terrain_col:
        excess = wind_col - terrain_col
        for line in wind_data: wind_data_changed.append(line[:-excess])
        wind_data = wind_data_changed
    elif wind_col < terrain_col: print('ERROR: wind data lacking collumns')

    if wind_row > terrain_row:
        excess = wind_row - terrain_row
        wind_data_changed = wind_data[:-excess]
        wind_data = wind_data_changed
    elif wind_col < terrain_col: print('ERROR: wind data lacking rows')

    # Change metadata
    num_digits_col = len(str(wind_col))
    num_digits_row = len(str(wind_row))
    wind_metadata[0] = wind_metadata[0][:-(num_digits_col + 1)] + str(len(wind_data)) +'\n'
    wind_metadata[1] = wind_metadata[1][:-(num_digits_row + 1)] + str(len(wind_data[0])) +'\n'
    return wind_metadata, wind_data
wind_vel_metadata, wind_vel_data = matchWindWithTerrain(terrain_dimensions, wind_vel_metadata, wind_vel_data)
wind_ang_metadata, wind_ang_data = matchWindWithTerrain(terrain_dimensions, wind_ang_metadata, wind_ang_data)

def writeWind(wind_metadata, wind_data, wind_dir):
    with open(wind_dir, 'w') as file:
        for line in wind_metadata:
            file.write(line)
        for line in wind_data:
            file.write(' ' + ' '.join(map(str, line)) + '\n')
writeWind(wind_vel_metadata, wind_vel_data, wind_vel_dir)
writeWind(wind_ang_metadata, wind_ang_data, wind_ang_dir)
