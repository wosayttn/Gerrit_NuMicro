#!/bin/bash

source ./config.sh

# ---------- Parameter Settings ----------
LIMIT=1000                              # Maximum number of runs to query at a time

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


gh run list --limit $LIMIT --json databaseId --jq '.[].databaseId' | while read run_id; do
    echo "Deleting workflow run: $run_id"
    gh run delete "$run_id"
done
