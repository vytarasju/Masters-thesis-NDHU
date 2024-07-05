#!/bin/bash
#Path to the tar.gz folder
converted_folder="./converted"
wind_folder="./wind"

#Check if filename is provided as argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

check_and_create_folder() {
    local folder_name="$1"
    if [ ! -d "$folder_name" ]; then
        echo "Error: $folder_name folder not found in the current directory."
        mkdir -p "$folder_name"
        echo "$folder_name folder created"
        exit 1
    fi
}

check_and_create_folder "$converted_folder"
check_and_create_folder "$wind_folder"

filename=$(basename -- "$1")
xyz_filename=$(echo "$filename" | awk -F_ '{print $1 ".xyz"}')
xyz_dir="${converted_folder}/${xyz_filename}"
asc_dir="${converted_folder}/terrain.asc"

echo "$xyz_dir"

gdal_translate "$xyz_dir" "$asc_dir"

python reshape_wind.py "terrain.asc" "${filename}_vel.asc" "${filename}_ang.asc"

rm -r "$asc_dir"

