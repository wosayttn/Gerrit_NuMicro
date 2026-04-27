#!/bin/bash

set -e
set -x

source ./config.sh

# Read all departments
departments=$(jq -r 'keys[]' "$JSON_FILE")

for dept in $departments; do
    echo "Department: $dept"
    
    # Read the BSP list for this department
    bsp_list=$(jq -r --arg d "$dept" '.[$d] | keys[]' "$JSON_FILE")
    
    for bsp in $bsp_list; do
        echo "  BSP: $bsp"
        workflows=$(jq -r --arg d "$dept" --arg b "$bsp" '.[$d][$b].Workflow[]?' "$JSON_FILE")
        for wf in $workflows; do
            echo "    Workflow: $wf"
        done
    done
    
    echo "-------------------------"
done
