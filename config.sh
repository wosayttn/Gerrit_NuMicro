#!/bin/bash

# ---------- Settings ----------

# Board Support Packages to process
BSP_LIST=("NUC121" "M253" "M251" "M55M1" "M5531" "M3351")

# GitHub repository details, Use personal Access Tokens(PAT)
# GitHub has deprecated personal password authentication and replaced it with Personal Access Tokens (PATs). 
# Therefore, whether you are using the GitHub API or communicating with GitHub through the command line, you must use a PAT.
#
# https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens
#
# You can create a PAT in your GitHub account settings: 
#     Settings > < >Developer settings > Personal access tokens > Tokens (classic) > Generate new token
# For security reasons, avoid hardcoding the token directly in scripts. Instead, consider using environment
# Make sure to grant the necessary permissions to the token, such as repo access for private repositories.
GITHUB_TOKEN="ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# GitHub repository to push to
GITHUB_REPO="wosayttn/Gerrit_NuMicro"

# Source and Destination repository bases
SRC_GIT_BASE="http://wclin@10.1.8.206/p"
DST_REPO_URL="https://${GITHUB_TOKEN}@github.com/${GITHUB_REPO}.git"

export GITHUB_TOKEN=${GITHUB_TOKEN}
