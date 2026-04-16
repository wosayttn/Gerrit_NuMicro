#!/bin/bash

usage() {
    echo "Usage: $0 [--commit]"
    echo "  --commit   Stage SampleCode changes, commit, and push to refs/for/master"
}

DO_COMMIT=0

while [ $# -gt 0 ]; do
    case "$1" in
        --commit)
            DO_COMMIT=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

# 1. Collect all Python check scripts matching the naming rule in current directory
# Use nullglob to avoid treating unmatched wildcards as literal strings
shopt -s nullglob
# Match all .py scripts with names starting with Append...Check...
CHECK_SCRIPTS=( "$(pwd)"/Append*Check*.py )
shopt -u nullglob

# Verify that at least one script was found
if [ ${#CHECK_SCRIPTS[@]} -eq 0 ]; then
    echo "[ERROR] No Python CHECK_SCRIPTs found in $(pwd)!"
    exit 1
fi

echo "Found ${#CHECK_SCRIPTS[@]} Check Scripts to run:"
for script in "${CHECK_SCRIPTS[@]}"; do
    echo "  - $(basename "$script")"
done

if [ "$DO_COMMIT" -eq 1 ]; then
    echo "Git add/commit/push: enabled"
else
    echo "Git add/commit/push: disabled (dry-run mode)"
fi

echo "-------------------------------------------------------"
echo "Starting automated updates for all project directories..."
echo "-------------------------------------------------------"

# Use set +e if you want the loop to continue even if one project fails to pull/push
set -e

for dir in */; do
    dir=${dir%/}

    # Skip hidden directories
    [[ "$dir" == .* ]] && continue

    echo "[PROCESSING] Project: $dir"

    if cd "$dir"; then
        # 1. Discard changes to tracked files (like modified .uvprojx or .cproject)
        git restore . 
        # 2. Force clean untracked/ignored files and folders
        git clean -ffdx
        git pull --rebase

        # 3. Run all discovered Python modification scripts in sequence
        for APP_SCRIPT in "${CHECK_SCRIPTS[@]}"; do
            if [ -f "$APP_SCRIPT" ]; then
                echo "  -> Running check script: $(basename "$APP_SCRIPT")"
                # Pass the current directory to the python script
                python3 "$APP_SCRIPT" .
            else
                echo "  [SKIP] Python script not found: $APP_SCRIPT"
            fi
        done

        if [ "$DO_COMMIT" -eq 1 ]; then
            # 4. Stage ONLY tracked files (prevents adding generated files)
            git add -u "SampleCode/"

            # Check if there are actually changes to commit
            if ! git diff --cached --exit-code > /dev/null; then
                git commit -m "[MDK5/CSolution/Eclipse/IAR] Append no-unaligned-access flag."

                # 5. Gerrit Hook & Change-Id Generation
                # Download hook if missing to ensure Change-Id is generated
                if [ ! -f ".git/hooks/commit-msg" ]; then
                    curl -Lo .git/hooks/commit-msg http://10.1.8.206/tools/hooks/commit-msg
                    chmod +x .git/hooks/commit-msg
                    # Amend to trigger the hook and add Change-Id
                    git commit --amend --no-edit
                fi

                # 6. Push to Gerrit
                echo "  -> Pushing to Gerrit..."
                git push origin HEAD:refs/for/master
            else
                echo "  [INFO] No changes detected, skipping commit."
            fi
        else
            echo "  [INFO] Commit skipped (run with --commit to enable git add/commit/push)."
        fi

        # Return to root
        cd ..
        echo "[DONE] Finished $dir"
    else
        echo "[ERROR] Could not enter directory: $dir"
    fi
    echo "-------------------------------------------------------"
done

echo "All projects have been processed."
