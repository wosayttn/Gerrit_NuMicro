#!/bin/bash
set -e

source ./config.sh   # must define JSON_FILE

README_FILE="README.md"

# Define workflow set
WORKFLOWS=("NuEclipse" "VSCode" "IAR" "MDK5" "CodeAnalysis")

# Start README
{
  echo "# NuMicro BSP Status"
  echo
} > "$README_FILE"

# Table Header
{
  printf "| BSP "
  for wf in "${WORKFLOWS[@]}"; do
      printf "| %s " "$wf"
  done
  printf "|\n"

  printf "|-----"
  for _ in "${WORKFLOWS[@]}"; do
      printf "|-----------"
  done
  printf "|\n"
} >> "$README_FILE"

# Process each department and BSP entries
for dept in $(jq -r 'keys[]' "$JSON_FILE"); do

  for bsp in $(jq -r --arg d "$dept" '.[$d][]' "$JSON_FILE"); do

      branch="${dept}_${bsp}"

      printf "| %s " "$branch" >> "$README_FILE"

      for wf in "${WORKFLOWS[@]}"; do
          printf "| ![](https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows/%s.yml/badge.svg?branch=%s) " "$wf" "$branch" >> "$README_FILE"
      done

      printf "|\n" >> "$README_FILE"
  done

done

echo "✅ README.md generated."
