#!/bin/bash

# 1. 抓取當前目錄下所有符合命名規則的 Python Check Scripts
# 使用 nullglob 避免找不到檔案時將萬用字元當成字串處理
shopt -s nullglob
# 這裡設定萬用字元比對所有 AppendUninitializedVariableCheck 開頭的 .py 檔
CHECK_SCRIPTS=( "$(pwd)"/AppendUninitializedVariableCheck*.py )
shopt -u nullglob

# 檢查是否有找到任何腳本
if [ ${#CHECK_SCRIPTS[@]} -eq 0 ]; then
    echo "[ERROR] No Python CHECK_SCRIPTs found in $(pwd)!"
    exit 1
fi

echo "Found ${#CHECK_SCRIPTS[@]} Check Scripts to run:"
for script in "${CHECK_SCRIPTS[@]}"; do
    echo "  - $(basename "$script")"
done

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
        # 2. Remove new untracked files/folders
        git clean -fd
        git pull --rebase

        # 3. 依序執行所有找到的 Python Modification Scripts
        for APP_SCRIPT in "${CHECK_SCRIPTS[@]}"; do
            if [ -f "$APP_SCRIPT" ]; then
                echo "  -> Running check script: $(basename "$APP_SCRIPT")"
                # Pass the current directory to the python script
                python3 "$APP_SCRIPT" .
            else
                echo "  [SKIP] Python script not found: $APP_SCRIPT"
            fi
        done

        # 4. Stage ONLY tracked files (prevents adding .bak files)
        git add -u "SampleCode/"
        
        # Check if there are actually changes to commit
        if ! git diff --cached --exit-code > /dev/null; then
            git commit -m "[MDK5/CSolution/Eclipse] Append uninitialized variable checking."
            
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

        # Return to root
        cd ..
        echo "[DONE] Finished $dir"
    else
        echo "[ERROR] Could not enter directory: $dir"
    fi
    echo "-------------------------------------------------------"
done

echo "All projects have been processed."
