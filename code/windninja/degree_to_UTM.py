import sys
import re
import math
from pyproj import CRS, Transformer

path_windninja = "/home/vytska/thesis/code/windninja/"

# Check if arguments are provided
if len(sys.argv) < 1:
    sys.exit("No input provided")

input_string = sys.argv[1]
input_split = input_string.split('.')

if input_split[1] != "asc":
    sys.exit("Wrong extension file provided")

def readASCFile(filename):
        data_list = []
        metadata = []
        with open(path_windninja + "tif/" + filename, 'r') as f:
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

longitude_str = re.findall(r"[-+]?\d*\.\d+|\d+", metadata[2])[0]
laditude_str = re.findall(r"[-+]?\d*\.\d+|\d+", metadata[3])[0]
longitude = float(longitude_str)
laditude = float(laditude_str)

def get_utm_zone_and_hemisphere(lat, lon):
    """Determine the UTM zone and hemisphere from latitude and longitude."""
    # UTM zones range from 1 to 60 and are 6 degrees wide
    utm_zone = int((lon + 180) // 6) + 1

    # Hemisphere is 'north' if latitude is positive, otherwise 'south'
    hemisphere = 'north' if lat >= 0 else 'south'
    
    return utm_zone, hemisphere

def get_epsg_code(lat, lon):
    """Determine the EPSG code for the UTM zone and hemisphere."""
    utm_zone, hemisphere = get_utm_zone_and_hemisphere(lat, lon)
    
    # EPSG code for WGS84 UTM zones: 32600 + zone for northern hemisphere
    # and 32700 + zone for southern hemisphere
    if hemisphere == 'north':
        epsg_code = 32600 + utm_zone
    else:
        epsg_code = 32700 + utm_zone
    
    return epsg_code

def convert_to_utm(lat, lon):
    """Convert latitude and longitude to UTM coordinates."""
    epsg_code = get_epsg_code(lat, lon)
    
    # Define the WGS84 coordinate system
    wgs84 = CRS("EPSG:4326")

    # Define the UTM coordinate system using the EPSG code
    utm = CRS(f"EPSG:{epsg_code}")

    # Create a transformer object
    transformer = Transformer.from_crs(wgs84, utm, always_xy=True)

    # Convert from WGS84 to UTM
    easting, northing = transformer.transform(lon, lat)

    return easting, northing, epsg_code

easting, northing, epsg = convert_to_utm(lat=laditude, lon=longitude)

#Printing out for the bash script to take that as input
print(epsg)

#Change Metadata first two lines
num_digits_lon = len(longitude_str)
num_digits_lat = len(laditude_str)
metadata[2] = metadata[2][:-(num_digits_lon + 1)] + str(easting) +'\n'
metadata[3] = metadata[3][:-(num_digits_lat + 1)] + str(northing) +'\n'

num_digits_cell_size = len(re.findall(r"[-+]?\d*\.\d+|\d+", metadata[4])[0])
metadata[4] = metadata[4][:-(num_digits_cell_size + 1)] + '30' +'\n'

# Write back into ASC file
with open(path_windninja + 'tif/' + input_string, 'w') as file:
    for line in metadata:
        file.write(line)
    for line in terrain_data:
        file.write(' ' + ' '.join(map(str, line)) + '\n')