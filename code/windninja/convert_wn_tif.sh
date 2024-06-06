#!/bin/bash

# Check if filename is provided as argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

# Path to the tif folder
tif_folder="./tif"

# Create tif folder if it is not present
mkdir -p "$tif_folder"

# Extracting filename and extension
filename=$(basename -- "$1")
extension="${filename##*.}"
filename_no_ext="${filename%.*}"

# Path to the input file in the tif folder
input_file="${tif_folder}/${filename}"

# Check if the input file exists
if [ ! -f "$input_file" ]; then
    echo "Error: Input file $input_file not found in the tif folder."
    exit 1
fi

# Directory for output files
output_dir="./converted"

# Create output directory if it doesn't exist
mkdir -p "$output_dir"

# Path to intermediate file
intermediate_file="${tif_folder}/intermediate.asc"

# Output filenames
output_asc="${tif_folder}/${filename_no_ext}.asc"
output_xyz="${output_dir}/${filename_no_ext}.xyz"

# Run gdal_translate commands
gdal_translate -of AAIGrid "$input_file" "$output_asc"

python reshape_terrain.py "${filename_no_ext}.asc"

gdal_translate -of XYZ "$intermediate_file" "$output_xyz"

rm -r "${tif_folder}/${filename_no_ext}.asc.aux.xml"
rm -r "${tif_folder}/${filename_no_ext}.prj"
rm -r "${tif_folder}/${filename_no_ext}.asc"
rm -r "${tif_folder}/intermediate.asc"


echo "Conversion complete. Output files:"
echo "$output_xyz"
