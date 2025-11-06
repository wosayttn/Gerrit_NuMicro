#!/bin/bash

set -e
set -x

source ./config.sh

# Output README file
README_FILE="README.md"

# Start README content
echo "# NuMicro BSP Status" > "$README_FILE"
echo "" >> "$README_FILE"

# Loop through all departments
for dept in $(jq -r 'keys[]' "$JSON_FILE"); do
    echo "## Department: $dept" >> "$README_FILE"
    echo "" >> "$README_FILE"

    # Markdown table header
    cat <<EOL >> "$README_FILE"
| BSP     | NuEclipse | VSCode | IAR | MDK5 |
|---------|-----------|--------|-----|------|
EOL

    # Loop through BSPs in this department
    bsp_list=$(jq -r --arg d "$dept" '.[$d][]' "$JSON_FILE")
    for bsp in $bsp_list; do
        echo "| $bsp | ![NuEclipse](https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows/Eclipse.yml/badge.svg?branch=${dept}_$bsp) | ![VSCode](https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows/VSCode.yml/badge.svg?branch=${dept}_$bsp) | ![IAR](https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows/IAR_SelfHosted.yml/badge.svg?branch=${dept}_$bsp) | ![MDK5](https://github.com/wosayttn/Gerrit_NuMicro/actions/workflows/MDK5.yml/badge.svg?branch=${dept}_$bsp) |" >> "$README_FILE"
    done

    echo "" >> "$README_FILE"
done

echo "README.md generated successfully!"
