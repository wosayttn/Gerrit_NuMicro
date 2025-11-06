#!/bin/bash
set -e

source ./config.sh   # must define JSON_FILE

README_FILE="README.md"

URL_PREFIX="https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows"

# Define workflow set
WORKFLOWS=("NuEclipse" "VSCode" "IAR" "MDK5" "CodeAnalysis" "PrjChk")

# Start README
{
  echo "# NuMicro BSP Status"
} > "$README_FILE"

# Table Header
{
  #printf "| BSP "
  #for wf in "${WORKFLOWS[@]}"; do
  #    printf "| %s " "$wf"
  #done
  #printf "|\n"

  #printf "|-----"
  #for _ in "${WORKFLOWS[@]}"; do
  #    printf "|-----------"
  #done
  #printf "|\n"
  
  printf "|Branch|Workflows|\n"
  printf "|-----|------|\n"

} >> "$README_FILE"

# Process each department and BSP entries
for dept in $(jq -r 'keys[]' "$JSON_FILE"); do

  for bsp in $(jq -r --arg d "$dept" '.[$d][]' "$JSON_FILE"); do

      branch="${dept}_${bsp}"

      printf "| %s |" "$branch" >> "$README_FILE"

      for wf in "${WORKFLOWS[@]}"; do
          printf "[![](%s)](%s)" \
            "$URL_PREFIX/${wf}.yml/badge.svg?branch=$branch" \
            "$URL_PREFIX/${wf}.yml?query=branch:$branch"  >> "$README_FILE"
      done

      printf "|\n" >> "$README_FILE"
  done

done

echo "✅ README.md generated."
