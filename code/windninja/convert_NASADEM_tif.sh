#!/bin/bash
#Path to the tif folder
NASADEM_folder="./NASADEM"

#Check if filename is provided as argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

#Check if the tif folder exists
if [ ! -d "$NASADEM_folder" ]; then
    echo "Error: NASADEM folder not found in the current directory."
    mkdir -p "$NASADEM_folder"
    echo "NASADEM folder created, upload your NASADEM.tar.gz file there"
    exit 1
else
    cd "$NASADEM_folder"
fi

filename=$(basename -- "$1")

if [[ "$filename" == *.tar.gz ]]; then
    tar -xzf "$filename"
    rm -r "$filename"
else
    echo "Error: non ".tar.gz" file provided"
    exit 1
fi

# Initialize a variable to hold the .tif filename
tif_filename=""

# Loop through files in the directory and find the first .tif file
for file in *.tif; do
    if [[ -f "$file" ]]; then
        tif_filename="$file"
        break
    fi
done

tif_folder="../tif"
# Create tif folder if it is not present
mkdir -p "$tif_folder"

# Check if a .tif file was found and save the filename
if [[ -n "$tif_filename" ]]; then
    echo "Found .tif file: $tif_filename"
    mv "$tif_filename" "$tif_folder"
else
    echo "No .tif file found in the directory."
    exit 1
fi

tif_folder="./tif"
cd ..
input_file="${tif_folder}/${tif_filename}"

# Directory for output files
output_dir="./converted"
# Create output directory if it doesn't exist
mkdir -p "$output_dir"

# Path to intermediate file
intermediate_file="${tif_folder}/intermediate.asc"

# Output filenames
filename_no_ext="${tif_filename%.*}"
output_asc="${tif_folder}/${filename_no_ext}.asc"
output_xyz="${output_dir}/${filename_no_ext}.xyz"
output_tif="${tif_folder}/${filename_no_ext}.tif"

# Run gdal_translate commands
gdal_translate -of AAIGrid "$input_file" "$output_asc"

python reshape_terrain.py "${filename_no_ext}.asc"
epsg_code=$(python degree_to_UTM.py intermediate.asc)

rm -r "${tif_folder}/${filename_no_ext}.asc.aux.xml"
rm -r "${tif_folder}/${filename_no_ext}.prj"
rm -r "${tif_folder}/${filename_no_ext}.asc"

gdal_translate -of XYZ "$intermediate_file" "$output_xyz"

gdal_translate -a_srs EPSG:"$epsg_code" "$intermediate_file" "$output_tif"

rm -r "$intermediate_file"

echo "Conversion complete. Output files:"
echo "$output_xyz"
