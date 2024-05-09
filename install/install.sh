#!/bin/bash

# Check if no arguments are provided
if [ $# -eq 0 ]; then
    echo -e "\e[31mSetup in progress, please use the other scripts.\e[0m"
    exit 1
fi

# Handle the -test argument
if [ "$1" == "-test" ]; then
    pip install -r ./requirements.txt
    python3 ./setup.py
else
    echo 
    exit 1
fi

