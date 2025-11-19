#!/bin/bash
set -e

source ./config.sh   # must define JSON_FILE

README_FILE="README.md"

URL_PREFIX="https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows"

# Define workflow set
WORKFLOWS=("NuEclipse" "VSCode" "IAR" "MDK5" "CodeAnalysis")

# Start README
{
  echo "# NuMicro BSPs"
  echo "## Current Status"
} > "$README_FILE"

# Table Header
{
  printf "| BSP "
  for wf in "${WORKFLOWS[@]}"; do
      printf "|%s" "$wf"
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

      printf "|%s" "$branch" >> "$README_FILE"

      for wf in "${WORKFLOWS[@]}"; do
          printf "|[![](%s)](%s)" \
            "$URL_PREFIX/${wf}.yml/badge.svg?branch=$branch" \
            "$URL_PREFIX/${wf}.yml?query=branch:$branch"  >> "$README_FILE"
      done

      printf "|\n" >> "$README_FILE"
  done

done

# Fetch runners JSON from GitHub
RUNNERS_JSON=$(gh api -H "Accept: application/vnd.github+json" /repos/$GITHUB_REPO/actions/runners)

# Start README
{
  echo "## Self-Hosted Runner List"

  # Start the Markdown table
  echo "| NAME | LABELS |"

  echo "|--------|--------|"

} >> "$README_FILE"

# Populate table rows
echo "$RUNNERS_JSON" | jq -r '.runners[] | "| \(.name) | \(.labels | map(.name) | join(", ")) |"' >> "$README_FILE"

echo "✅ README.md generated."
