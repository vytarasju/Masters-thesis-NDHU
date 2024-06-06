import sys
path_windninja = "/home/vytska/Masters-thesis-NDHU/code/windninja/"

# Check if arguments are provided
if len(sys.argv) < 1:
    sys.exit("No input provided")

input_string = sys.argv[1]
print(f"Reshaping {input_string}")
input_split = input_string.split('.')

if input_split[1] != "asc":
    sys.exit("Wrong extension file provided")

def readASCFile(filename):
        data_list = []
        metadata = []
        with open(path_windninja + "terrain-testing/" + filename, 'r') as f:
            # Skip the first 6 lines
            for index, line in enumerate(f):
                metadata.append(line)
                if index == 5: break
            for index, line in enumerate(f):
                line = line.split()
                if index == 0: line[0] = line[0].split('.')[0]
                data_list.append(line)
        return data_list, metadata

#Read ASC
terrain_data, metadata = readASCFile(input_string)
num_lines = len(terrain_data)
num_rows = len(terrain_data[0])

#Reshape ASC based on which part has the excess
if num_rows > num_lines:
    terrain_reshaped = []
    excess = num_rows - num_lines
    for line in terrain_data:
        terrain_reshaped.append(line[:-excess])
    terrain_data  = terrain_reshaped
if num_rows < num_lines: 
    excess = num_lines - num_rows
    terrain_data = terrain_data[:-excess]

#Change Metadata first two lines
num_digits_lines = len(str(num_lines))
num_digits_rows = len(str(num_rows))
metadata[0] = metadata[0][:-(num_digits_lines + 1)] + str(len(terrain_data)) +'\n'
metadata[1] = metadata[1][:-(num_digits_rows + 1)] + str(len(terrain_data[0])) +'\n'

# Write back into ASC file
with open(path_windninja + 'terrain-testing/' + 'intermediate.asc', 'w') as file:
    for line in metadata:
        file.write(line)
    for line in terrain_data:
        file.write(' ' + ' '.join(map(str, line)) + '\n')