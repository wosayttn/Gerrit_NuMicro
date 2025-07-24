#!/bin/bash

# ---------- Parameter Settings ----------
REPO="wosayttn/Gerrit_NuMicro"         # Please modify this to your own repo
WORKFLOW_NAME="VSCode.yml IAR.yml Eclipse.yml"  # Please modify this to your workflow file names or IDs
LIMIT=100                              # Maximum number of runs to query at a time

# ---------- Validate Parameters ----------
if [[ "$REPO" == "your-username/your-repo" ]]; then
    echo "⚠️ Please set REPO to your own GitHub repository (format: username/repo)"
    exit 1
fi

# ---------- Check if gh CLI is installed ----------
if ! command -v gh &> /dev/null; then
    echo "❌ GitHub CLI (gh) is not installed. Please install it first!"
    exit 1
fi

# ---------- Ensure gh is logged in ----------
if ! gh auth status &> /dev/null; then
    echo "🔐 GitHub CLI is not logged in. Please run: gh auth login"
    exit 1
fi

for WN in $WORKFLOW_NAME; do

    # ---------- Validate workflow exists ----------
    if ! gh workflow view "$WN" --repo "$REPO" &> /dev/null; then
        echo "❌ Cannot find workflow '$WN'. Please check the name."
        exit 1
    fi

    # ---------- Begin deletion ----------
    echo "📋 Searching for failed runs of ${WN} (up to $LIMIT entries)..."

    COUNT=0
    while read -r RUN_ID; do
        echo "🗑️ Deleting failed run ID: $RUN_ID"
        gh run delete "$RUN_ID" --repo "$REPO"
        ((COUNT++))
    done < <(gh run list --workflow="$WN" --limit "$LIMIT" --repo "$REPO" --json databaseId,conclusion -q '.[] | select(.conclusion == "failure" or .conclusion == "cancelled" or .conclusion == "success") | .databaseId')
                 
    echo "✅ Cleanup complete. Deleted $COUNT failed workflow runs."

done
