#!/bin/bash

set -e
set -x

export PATH="$HOME/.local/bin:$PATH"

BSP_LIST=("NUC121" "M253" "M251" "M55M1" "M5531" "M3351")
#BSP_LIST=("M55M1" "M5531")
#BSP_LIST=("NUC121" "M253" "M251")

export GITHUB_TOKEN="ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

SRC_GIT_BASE="http://wclin@10.1.8.206/p"
DST_GITHUB_BASE="https://github.com/wosayttn"

if [ -z "$GITHUB_TOKEN" ]; then
  echo "❌ Please set GITHUB_TOKEN environment variable first."
  exit 1
fi

#echo "Installing git-filter-repo..."
#pip3 install --quiet git-filter-repo --break-system-packages

PATH_SCRIPT=`pwd`

for BSP in "${BSP_LIST[@]}"; do
  echo "======================================"
  echo "🔄 Processing $BSP BSP..."

  SRC_REPO_URL="${SRC_GIT_BASE}/${BSP}/bsp.git"
  DST_REPO_URL="https://${GITHUB_TOKEN}@github.com/wosayttn/Gerrit_NuMicro.git"
  LOCAL_DIR="${BSP}BSP"

  rm -rf ${LOCAL_DIR}
  
  echo "📥 Cloning mirror from $SRC_REPO_URL..."
  #git clone --mirror "$SRC_REPO_URL" "$LOCAL_DIR"
  git clone  "$SRC_REPO_URL" "$LOCAL_DIR"

  cp -af .github "$LOCAL_DIR"/.github

  cd "$LOCAL_DIR"
  # Find and replace
  NEW_FILE="$PATH_SCRIPT/vcpkg-configuration.json"
  if [ -f "$NEW_FILE" ]; then
    find . -type f -name "vcpkg-configuration.json" | while read -r FILE; do
      echo "Replacing $FILE"
      echo "cp $NEW_FILE $FILE"
      cp -af "$NEW_FILE" "$FILE"
    done

    git add .
  fi

  git add -f .github/workflows/requirements.txt
  git add .github && git diff --cached --quiet || git commit -m "Add/update .github folder"

  echo "🧹 Removing Document/*.chm from history..."
  git filter-repo --path-glob "Document/*.chm" --invert-paths --force

  echo "🧹 Removing _xxxx from history..."

  find . -type d -name '_*'
  if [[ "$BSP" == M55* ]]; then
	git filter-repo \
                --path-glob "Document/*.chm" \
                --path-glob '_*/' \
		--path-glob 'ThirdParty/_tflite_micro_EI' \
                --path-glob 'Library/_*/' \
		--path-glob 'Library/**/_*/' \
                --path-glob 'SampleCode/_*/' \
                --invert-paths --force
  else
	git filter-repo \
		--path-glob "Document/*.chm" \
		--path-glob '**/_*/' \
		--path-glob '_*/' \
		--invert-paths --force
  fi
  find . -type d -name '_*'

  git reset $(git commit-tree HEAD^{tree} -m "Commit at $(date -u +"%Y-%m-%dT%H:%M:%SZ")") --hard

  echo "🔗 Setting remote URL to GitHub: $DST_REPO_URL"
  #git remote remove origin
  git remote add origin "$DST_REPO_URL"

  echo "🚀 Pushing cleaned repo to GitHub branch: $TARGET_BRANCH..."
  TARGET_BRANCH="${BSP}_master"
  git checkout -B "$TARGET_BRANCH"  # create local branch if needed

  git push -u origin HEAD:"$TARGET_BRANCH" -f

  cd ..

  echo "✅ Done with $BSP"

done

echo "🎉 All BSPs cleaned and pushed to GitHub successfully."
