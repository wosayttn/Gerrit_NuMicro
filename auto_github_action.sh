#!/bin/bash

set -e
set -x

source ./config.sh

export PATH="$HOME/.local/bin:$PATH"

if [ -z "$GITHUB_TOKEN" ]; then
  echo "❌ Please set GITHUB_TOKEN environment variable first."
  exit 1
fi

# Submodule handling mode:
#   "auto"    - Keep public HTTPS submodules (rewrite git@github.com to https://);
#               flatten internal Gerrit / relative-path submodules into regular files
#   "flatten" - Flatten (inline) all submodules into regular files
#   "keep"    - Keep all submodules as gitlinks
SUBMODULE_MODE="${SUBMODULE_MODE:-auto}"

#echo "Installing git-filter-repo..."
#pip3 install --quiet git-filter-repo --break-system-packages

# Normalize GitHub SSH URLs to HTTPS URLs
normalize_github_url() {
  local url="$1"
  if [[ "$url" =~ ^git@github\.com:(.+)$ ]]; then
    echo "https://github.com/${BASH_REMATCH[1]}"
  elif [[ "$url" =~ ^ssh://git@github\.com/(.+)$ ]]; then
    echo "https://github.com/${BASH_REMATCH[1]}"
  else
    echo "$url"
  fi
}

# Return 0 (true) if the submodule with given URL should be flattened into regular files,
# or 1 (false) if it should be kept as a Git submodule gitlink.
should_flatten_submodule() {
  local sub_url="$1"
  if [ "$SUBMODULE_MODE" = "flatten" ]; then
    return 0
  elif [ "$SUBMODULE_MODE" = "keep" ]; then
    return 1
  fi

  # auto mode: normalize git@github.com -> https://github.com first
  sub_url=$(normalize_github_url "$sub_url")

  # Flatten relative URLs, internal Gerrit URLs, private/loopback IPs, or non-HTTPS URLs
  if [[ "$sub_url" =~ ^\.\.?/ ]] || \
     [[ -n "$SRC_GIT_BASE" && "$sub_url" == "${SRC_GIT_BASE}"* ]] || \
     [[ "$sub_url" =~ ^https?://([^/@]+@)?(localhost|127\.|10\.|192\.168\.|172\.(1[6-9]|2[0-9]|3[0-1])\.) ]] || \
     [[ ! "$sub_url" =~ ^https:// ]]; then
    return 0
  fi

  return 1
}

# Check if GERRIT_DIR has any submodule that needs flattening and is uninitialized/outdated
gerrit_submodules_need_update() {
  if [ ! -f .gitmodules ]; then
    return 1
  fi

  while IFS= read -r -d '' entry; do
    local key="${entry%%$'\n'*}"
    local sub_path="${entry#*$'\n'}"
    sub_path="${sub_path%$'\r'}"
    local sub_name="${key#submodule.}"
    sub_name="${sub_name%.path}"
    local sub_url
    sub_url=$(git config -f .gitmodules --get "submodule.${sub_name}.url" || echo "")
    sub_url="${sub_url%$'\r'}"

    if should_flatten_submodule "$sub_url"; then
      if git submodule status --recursive -- "$sub_path" 2>/dev/null | grep -q '^[-+U]'; then
        return 0
      fi
    fi
  done < <(git config -z -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)

  return 1
}

# Sync and update submodules in GERRIT_DIR that need to be flattened (and deinit kept ones so they stay clean)
sync_gerrit_submodules() {
  if [ ! -f .gitmodules ]; then
    return 0
  fi

  git submodule sync --recursive || true

  while IFS= read -r -d '' entry; do
    local key="${entry%%$'\n'*}"
    local sub_path="${entry#*$'\n'}"
    sub_path="${sub_path%$'\r'}"
    local sub_name="${key#submodule.}"
    sub_name="${sub_name%.path}"
    local sub_url
    sub_url=$(git config -f .gitmodules --get "submodule.${sub_name}.url" || echo "")
    sub_url="${sub_url%$'\r'}"

    if ! git ls-files --stage -- "$sub_path" | awk '$1 == "160000" {found=1} END {exit !found}'; then
      continue
    fi

    if should_flatten_submodule "$sub_url"; then
      echo "🔄 Fetching submodule '$sub_path' ($sub_url) for flattening..."
      git \
        -c url."https://github.com/".insteadOf="git@github.com:" \
        -c url."https://github.com/".insteadOf="ssh://git@github.com/" \
        submodule update --init --recursive --force -- "$sub_path"
    else
      # Kept public submodule: ensure no stale checkout remains on disk
      git submodule deinit -f -- "$sub_path" 2>/dev/null || true
    fi
  done < <(git config -z -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)
}

# Process submodules in GITHUB_DIR before modifying files / filtering history:
# - Rewrites GitHub SSH submodule URLs to HTTPS so GitHub Actions can clone them.
# - Flattens unreachable (internal Gerrit / private IP / relative URL) submodules into regular files,
#   or flattens all submodules when SUBMODULE_MODE="flatten".
# - Deinits kept public submodules on disk so subsequent 'find' / 'git add .' will not dirty or revert them.
process_submodules_for_github() {
  if [ ! -f .gitmodules ]; then
    return 0
  fi

  echo "📦 Processing submodules (mode: ${SUBMODULE_MODE})..."

  while IFS= read -r -d '' entry; do
    local key="${entry%%$'\n'*}"
    local sub_path="${entry#*$'\n'}"
    sub_path="${sub_path%$'\r'}"
    local sub_name="${key#submodule.}"
    sub_name="${sub_name%.path}"

    local sub_url
    sub_url=$(git config -f .gitmodules --get "submodule.${sub_name}.url" || echo "")
    sub_url="${sub_url%$'\r'}"

    # Remove stale .gitmodules entries that have no matching path in worktree or index
    if [ ! -e "$sub_path" ] && ! git ls-files --error-unmatch -- "$sub_path" >/dev/null 2>&1; then
      echo "🧹 Removing stale .gitmodules entry: $sub_name ($sub_path)"
      git config -f .gitmodules --remove-section "submodule.${sub_name}" 2>/dev/null || true
      continue
    fi

    if should_flatten_submodule "$sub_url"; then
      echo "📂 Flattening submodule '$sub_path' (URL: $sub_url) into regular files..."
      git update-index --force-remove -- "$sub_path" || true
      git config -f .gitmodules --remove-section "submodule.${sub_name}" 2>/dev/null || true
      if [ -d "$sub_path" ]; then
        find "$sub_path" -name ".git" -prune -exec rm -rf {} +
        find "$sub_path" -name ".gitmodules" -exec rm -f {} +
        git add -f -- "$sub_path"
      fi
      rm -rf ".git/modules/${sub_name}"
    else
      if [ "$SUBMODULE_MODE" = "auto" ]; then
        local new_url
        new_url=$(normalize_github_url "$sub_url")
        if [ "$new_url" != "$sub_url" ]; then
          sub_url="$new_url"
          git config -f .gitmodules "submodule.${sub_name}.url" "$sub_url"
          echo "🔄 Rewrote SSH submodule URL to HTTPS for $sub_path: $sub_url"
        fi
      fi
      echo "🔗 Keeping public submodule '$sub_path' (URL: $sub_url)"
      git submodule deinit -f -- "$sub_path" 2>/dev/null || true
      rm -rf "${sub_path}/.git"
      mkdir -p "$sub_path"
    fi
  done < <(git config -z -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)

  if [ -f .gitmodules ]; then
    if ! git config -f .gitmodules --get-regexp '^submodule\.' >/dev/null 2>&1; then
      git rm -f --ignore-unmatch .gitmodules
    else
      git add .gitmodules
    fi
  fi

  if ! git diff --cached --quiet; then
    git commit -m "Process submodules for GitHub sync"
  fi
}

# Ensure .gitmodules and 160000 gitlinks remain 100% consistent after git filter-repo:
# 1. Remove any kept 160000 submodule gitlinks matching excluded '_' folder patterns.
# 2. Remove .gitmodules entries whose gitlink was stripped by filtering.
# 3. Remove orphan 160000 gitlinks that have no URL in .gitmodules (which break 'submodules: recursive').
cleanup_gitmodules_and_gitlinks() {
  # 1. Filter out kept 160000 gitlinks matching excluded '_' patterns (git filter-repo '_*/' misses slashless gitlinks)
  while IFS=$'\t' read -r meta sub_path; do
    [[ "$meta" == 160000* ]] || continue
    case "$sub_path" in
      _ThirdParty|_ThirdParty/*) ;;
      _*|ThirdParty/_tflite_micro_EI|ThirdParty/_tflite_micro_EI/*|Library/_*|Library/*/_*)
        echo "🧹 Removing excluded submodule gitlink: $sub_path"
        git update-index --force-remove -- "$sub_path"
        rm -rf -- "$sub_path"
        ;;
    esac
  done < <(git ls-files --stage)

  # 2. Remove .gitmodules sections whose 160000 gitlink no longer exists in the index
  if [ -f .gitmodules ]; then
    while IFS= read -r -d '' entry; do
      local key="${entry%%$'\n'*}"
      local sub_path="${entry#*$'\n'}"
      sub_path="${sub_path%$'\r'}"
      local sub_name="${key#submodule.}"
      sub_name="${sub_name%.path}"

      if ! git ls-files --stage -- "$sub_path" | awk '$1 == "160000" {found=1} END {exit !found}'; then
        echo "🧹 Removing filtered-out submodule '$sub_name' ($sub_path) from .gitmodules"
        git config -f .gitmodules --remove-section "submodule.${sub_name}" 2>/dev/null || true
      fi
    done < <(git config -z -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)
  fi

  # 3. Remove any orphan 160000 gitlinks that lack a valid URL in .gitmodules
  while IFS=$'\t' read -r meta sub_path; do
    [[ "$meta" == 160000* ]] || continue
    local has_mapping=false
    if [ -f .gitmodules ]; then
      while IFS= read -r -d '' entry; do
        local key="${entry%%$'\n'*}"
        local cfg_path="${entry#*$'\n'}"
        cfg_path="${cfg_path%$'\r'}"
        local sub_name="${key#submodule.}"
        sub_name="${sub_name%.path}"
        if [ "$cfg_path" = "$sub_path" ] && git config -f .gitmodules --get "submodule.${sub_name}.url" >/dev/null 2>&1; then
          has_mapping=true
          break
        fi
      done < <(git config -z -f .gitmodules --get-regexp '^submodule\..*\.path$' || true)
    fi

    if [ "$has_mapping" = false ]; then
      echo "⚠️ Removing orphan submodule gitlink without .gitmodules URL: $sub_path"
      git update-index --force-remove -- "$sub_path"
      rm -rf -- "$sub_path"
    fi
  done < <(git ls-files --stage)

  # 4. Remove .gitmodules if no submodule sections remain, otherwise stage it
  if [ -f .gitmodules ]; then
    if ! git config -f .gitmodules --get-regexp '^submodule\.' >/dev/null 2>&1; then
      git rm -f --ignore-unmatch .gitmodules
    else
      git add .gitmodules
    fi
  fi

  if ! git diff --cached --quiet; then
    git commit -m "Sync .gitmodules and submodule gitlinks after path filtering"
  fi
}

PATH_SCRIPT=$(pwd)

if [ ! -d "${PATH_SCRIPT}/GERRIT/" ]; then mkdir -p "${PATH_SCRIPT}/GERRIT/"; fi
if [ ! -d "${PATH_SCRIPT}/GITHUB/" ]; then mkdir -p "${PATH_SCRIPT}/GITHUB/"; fi

# Read all departments
DEPTS=$(jq -r 'keys[]' "$JSON_FILE")

for dept in $DEPTS; do
  cd "${PATH_SCRIPT}"

  # Read the BSP list for this department
  BSPS=$(jq -r --arg d "$dept" '.[$d] | keys[]' "$JSON_FILE")
  for bsp in $BSPS; do

    cd "${PATH_SCRIPT}"

    echo "======================================"
    echo "🔄 Processing $bsp BSP..."

    GERRIT_REPO_NAME=$(jq -r --arg d "$dept" --arg b "$bsp" '.[$d][$b].GerritRepoName' "$JSON_FILE")
    SRC_REPO_URL="${SRC_GIT_BASE}/${GERRIT_REPO_NAME}"
    GERRIT_DIR="${PATH_SCRIPT}/GERRIT/${dept}_${bsp}"
    GITHUB_DIR="${PATH_SCRIPT}/GITHUB/${dept}_${bsp}"

    if [ ! -d "${GERRIT_DIR}" ]; then
        echo "📥 Cloning mirror from $SRC_REPO_URL..."
        git clone "$SRC_REPO_URL" "$GERRIT_DIR"
        cd "${GERRIT_DIR}"
        sync_gerrit_submodules
    else
        cd "${GERRIT_DIR}"
        git reset --hard HEAD
        git clean -ffd
        if [ -f .gitmodules ]; then
            git submodule foreach --recursive 'git reset --hard && git clean -ffd' || true
        fi

        git fetch --prune
        LOCAL_HASH=$(git rev-parse HEAD)
        REMOTE_HASH=$(git rev-parse @{u} 2>/dev/null || echo "none")
        if [ "$REMOTE_HASH" = "none" ]; then
            echo "⚠️ No upstream branch set for $(basename "$GERRIT_DIR")"
            continue
        fi

        SUBMOD_OUTDATED=false
        if gerrit_submodules_need_update; then
            SUBMOD_OUTDATED=true
        fi

        if [ "$LOCAL_HASH" = "$REMOTE_HASH" ] && [ "$SUBMOD_OUTDATED" = false ]; then
            echo "✅ $bsp is already up to date. will skip the $bsp sync to github."
            continue
        else
            if [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
                git pull --rebase
                git clean -ffd
            fi
            sync_gerrit_submodules
        fi
    fi

    cd "${PATH_SCRIPT}"

    rm -rf "${GITHUB_DIR}"

    cp -af "${GERRIT_DIR}" "${GITHUB_DIR}"

    if [ -d "${GITHUB_DIR}/.github" ]; then
      rm -rf "${GITHUB_DIR}/.github"
    fi

    cp -af .github "${GITHUB_DIR}/.github"

    # BSP.workflow is exist or not. if not, remove "${GITHUB_DIR}/.github/workflows/xxx.yml"
    workflows=$(jq -r --arg d "$dept" --arg b "$bsp" '.[$d][$b].Workflow[]?' "${PATH_SCRIPT}/$JSON_FILE")
    for wf in "${WORKFLOWS[@]}"; do
        if ! echo "$workflows" | grep -qx "$wf"; then
            rm -f "${GITHUB_DIR}/.github/workflows/${wf}.yml"
        fi
    done

    rm -rf "${GITHUB_DIR}/.github/IAR_Nuvoton"

    cd "${GITHUB_DIR}"

    # Handle submodules first (flatten internal/relative ones, deinit kept public ones)
    process_submodules_for_github

    # Find and replace vcpkg-configuration.json (touches superproject & flattened submodules, skips kept submodules)
    NEW_FILE="$PATH_SCRIPT/vcpkg-configuration.json"
    if [ -f "$NEW_FILE" ]; then
      find . -not -path './.git/*' -type f -name "vcpkg-configuration.json" | while read -r FILE; do
        echo "Replacing $FILE"
        echo "cp $NEW_FILE $FILE"
        cp -af "$NEW_FILE" "$FILE"
      done

      git add .
    fi

    #git add -f .github/workflows/requirements.txt
    git add -f .github
    if ! git diff --cached --quiet; then
      git commit -m "Add/update .github folder"
    fi

    # Backup _ThirdParty (both regular files and any kept 160000 submodule gitlinks) before git filter-repo
    TMP_THIRDPARTY=""
    THIRDPARTY_GITLINKS=()
    if [ -d "_ThirdParty" ] || git ls-files --stage -- _ThirdParty | grep -q .; then
      TMP_THIRDPARTY=$(mktemp -d)
      if [ -d "_ThirdParty" ]; then
        cp -af "_ThirdParty" "${TMP_THIRDPARTY}/_ThirdParty"
      fi
      while IFS=$'\t' read -r meta sub_path; do
        [[ "$meta" == 160000* ]] || continue
        sub_sha=$(echo "$meta" | awk '{print $2}')
        THIRDPARTY_GITLINKS+=("${sub_sha}"$'\t'"${sub_path}")
      done < <(git ls-files --stage -- _ThirdParty)
    fi

    echo "🧹 Removing Document/*.chm and _xxxx from history..."
    find . -not -path './.git/*' -type d -name '_*'
    git filter-repo \
      --path-glob "Document/*.chm" \
      --path-glob '_*/' \
      --path-glob 'ThirdParty/_tflite_micro_EI' \
      --path-glob 'Library/_*/' \
      --path-glob 'Library/**/_*/' \
      --invert-paths --force

    # Restore _ThirdParty folder and any kept submodule gitlinks inside it
    if [ -n "$TMP_THIRDPARTY" ]; then
      if [ -d "${TMP_THIRDPARTY}/_ThirdParty" ]; then
        cp -af "${TMP_THIRDPARTY}/_ThirdParty" "${GITHUB_DIR}/_ThirdParty"
        git add -f _ThirdParty
      fi
      for gl in "${THIRDPARTY_GITLINKS[@]}"; do
        sub_sha="${gl%%$'\t'*}"
        sub_path="${gl#*$'\t'}"
        mkdir -p "$sub_path"
        git update-index --add --cacheinfo "160000,${sub_sha},${sub_path}"
      done
      rm -rf "$TMP_THIRDPARTY"
      if ! git diff --cached --quiet; then
        git commit -m "Restore _ThirdParty folder"
      fi
    fi

    # Clean up .gitmodules and excluded/orphan gitlinks after path filtering
    cleanup_gitmodules_and_gitlinks

    find . -not -path './.git/*' -type d -name '_*'

    git reset "$(git commit-tree HEAD^{tree} -m "Commit at $(date -u +"%Y-%m-%dT%H:%M:%SZ")")" --hard

    echo "🔗 Setting remote URL to GitHub: $DST_REPO_URL"
    if git remote get-url github >/dev/null 2>&1; then
      git remote set-url github "$DST_REPO_URL"
    else
      git remote add github "$DST_REPO_URL"
    fi

    TARGET_BRANCH="${dept}_${bsp}"
    echo "🚀 Pushing cleaned repo to GitHub branch: $TARGET_BRANCH..."
    git checkout -B "$TARGET_BRANCH"  # create local branch if needed

    git push -u github HEAD:"$TARGET_BRANCH" -f

    cd "${PATH_SCRIPT}"

    echo "✅ Done with $bsp"

  done

done

echo "🎉 All BSPs cleaned and pushed to GitHub successfully."
