#!/bin/bash

source ./config.sh

# ---------- Parameter Settings ----------
LIMIT=200                              # Maximum number of runs to query at a time

# ---------- Check if GITHUB_TOKEN is configuraed ----------
if [ -z "$GITHUB_TOKEN" ]; then
  echo "❌ Please set GITHUB_TOKEN environment variable first."
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

WORKFLOW_NAME=""
# ---------- list all workflow file in workflows folder ----------
for f in .github/workflows/*.yml; do
    [ -f "$f" ] && WORKFLOW_NAME="$WORKFLOW_NAME $(basename "$f")"
done

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
