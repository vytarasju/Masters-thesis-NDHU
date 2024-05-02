#!/bin/bash

# Check if filename is provided as argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

# Path to the tif folder
tif_folder="./tif"

# Check if the tif folder exists
if [ ! -d "$tif_folder" ]; then
    echo "Error: tif folder not found in the current directory."
    exit 1
fi

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

# Output filenames
output_asc="${output_dir}/${filename_no_ext}.asc"
output_xyz="${output_dir}/${filename_no_ext}.xyz"

# Run gdal_translate commands
gdal_translate -of AAIGrid "$input_file" "$output_asc"
gdal_translate -of XYZ "$input_file" "$output_xyz"

echo "Conversion complete. Output files:"
echo "$output_asc"
echo "$output_xyz"
