#!/bin/bash

set -e
set -x

source ./config.sh

# Read all departments
departments=$(jq -r 'keys[]' "$JSON_FILE")

for dept in $departments; do
    echo "Department: $dept"
    
    # Read the BSP list for this department
    bsp_list=$(jq -r --arg d "$dept" '.[$d][]' "$JSON_FILE")
    
    for bsp in $bsp_list; do
        echo "  BSP: $bsp"
    done
    
    echo "-------------------------"
done
