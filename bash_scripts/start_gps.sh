#!/bin/bash

# Ensure the Python script is in the same directory or provide the full path.
PYTHON_SCRIPT="/home/user/GPS/GPS_setup.py"

# Check if Python is installed
if ! command -v python3 &> /dev/null
then
    echo "Python3 is not installed. Please install Python3 to continue."
    exit 1
fi

# Run the Python script
echo "Running the Python script: $PYTHON_SCRIPT"
python3 "$PYTHON_SCRIPT"

# Check if the script ran successfully
if [ $? -eq 0 ]; then
    echo "Python script ran successfully."
else
    echo "Python script failed."
fi

