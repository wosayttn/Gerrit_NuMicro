#!/bin/bash

set -e
set -x

source ./config.sh

export PATH="$HOME/.local/bin:$PATH"

if [ -z "$GITHUB_TOKEN" ]; then
  echo "❌ Please set GITHUB_TOKEN environment variable first."
  exit 1
fi

#echo "Installing git-filter-repo..."
#pip3 install --quiet git-filter-repo --break-system-packages

PATH_SCRIPT=`pwd`

if [ ! -d "${PATH_SCRIPT}/GERRIT/" ]; then mkdir -p "${PATH_SCRIPT}/GERRIT/"; fi
if [ ! -d "${PATH_SCRIPT}/GITHUB/" ]; then mkdir -p "${PATH_SCRIPT}/GITHUB/"; fi

for BSP in "${BSP_LIST[@]}"; do

  cd ${PATH_SCRIPT}
 
  echo "======================================"
  echo "🔄 Processing $BSP BSP..."

  SRC_REPO_URL="${SRC_GIT_BASE}/${BSP}/bsp.git"
  GERRIT_DIR="${PATH_SCRIPT}/GERRIT/${BSP}BSP"
  GITHUB_DIR="${PATH_SCRIPT}/GITHUB/${BSP}BSP"

  if [ ! -d "${GERRIT_DIR}" ]; then 
      echo "📥 Cloning mirror from $SRC_REPO_URL..."
      git clone  "$SRC_REPO_URL" "$GERRIT_DIR"
  else
      cd ${GERRIT_DIR}
      git reset --hard HEAD
      git pull --rebase
      cd ${PATH_SCRIPT}
  fi

  rm -rf ${GITHUB_DIR}

  cp -af ${GERRIT_DIR} ${GITHUB_DIR}

  cp -af .github "${GITHUB_DIR}/.github"
  rm -rf "${GITHUB_DIR}/.github/IAR_Nuvoton"

  cd "${GITHUB_DIR}"
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
	#git filter-repo \
	#	--path-glob "Document/*.chm" \
	#	--path-glob '_*/' \
	#	--path-glob 'ThirdParty/_tflite_micro_EI' \
	#	--path-glob 'Library/_*/' \
	#	--path-glob 'Library/**/_*/' \
	#	--path-glob 'SampleCode/_*/' \
	#	--invert-paths --force

	git filter-repo \
		--path-glob "Document/*.chm" \
		--path-glob '_*/' \
		--path-glob 'ThirdParty/_tflite_micro_EI' \
	  --path-glob 'Library/_*/' \
		--path-glob 'Library/**/_*/' \
		--invert-paths --force

  else
	#git filter-repo \
	#	--path-glob "Document/*.chm" \
	#	--path-glob '**/_*/' \
	#	--path-glob '_*/' \
	#	--invert-paths --force

	git filter-repo \
		--path-glob "Document/*.chm" \
		--path-glob '_*/' \
		--path-glob 'ThirdParty/_*/' \
		--path-glob 'ThirdParty/**/_*/' \
		--path-glob 'Library/_*/' \
		--path-glob 'Library/**/_*/' \
		--invert-paths --force
  fi
  
  # _ThirdParty miss Workaround
  if [ -d "${GERRIT_DIR}/_ThirdParty" ]; then
	  cp -af "${GERRIT_DIR}/_ThirdParty" ${GITHUB_DIR}
	  git add _ThirdParty
          git commit -m "Restore _ThirdParty folder"
  fi

  find . -type d -name '_*'

  git reset $(git commit-tree HEAD^{tree} -m "Commit at $(date -u +"%Y-%m-%dT%H:%M:%SZ")") --hard

  echo "🔗 Setting remote URL to GitHub: $DST_REPO_URL"
  git remote add github "$DST_REPO_URL"

  echo "🚀 Pushing cleaned repo to GitHub branch: $TARGET_BRANCH..."
  TARGET_BRANCH="${BSP}_master"
  git checkout -B "$TARGET_BRANCH"  # create local branch if needed

  git push -u github HEAD:"$TARGET_BRANCH" -f

  cd ${PATH_SCRIPT}

  echo "✅ Done with $BSP"

done

echo "🎉 All BSPs cleaned and pushed to GitHub successfully."
