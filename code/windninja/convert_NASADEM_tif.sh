#!/bin/bash

#Check if filename is provided as argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

#Path to the tif folder
NASADEM_folder="./NASADEM"

#Check if the tif folder exists
if [ ! -d "$NASADEM_folder" ]; then
    echo "Error: NASADEM folder not found in the current directory."
    exit 1
fi

filename=$(basename -- "$1")

extension="${filename##*.}"
filename_no_ext="${filename%.*}"

