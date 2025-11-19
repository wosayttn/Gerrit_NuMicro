#!/bin/bash

set -x

source ./config.sh

# Gerrit server settings
GERRIT_HOST="10.1.8.206"
GERRIT_PORT="29418"
GERRIT_USER="sasms00jenkins"

# Verify JSON
jq empty "$JSON_FILE" || { echo "Invalid JSON"; exit 1; }

# Loop through each group
for group in $(jq -r 'keys[]' "$JSON_FILE"); do

    echo "Fetching $group ..."

    # Fetch emails from Gerrit group members
    emails=$(ssh -p "$GERRIT_PORT" \
        -oKexAlgorithms=+diffie-hellman-group14-sha1 \
        -oHostKeyAlgorithms=+ssh-rsa \
        -oPubkeyAcceptedAlgorithms=+ssh-rsa \
        "$GERRIT_USER@$GERRIT_HOST" \
        "gerrit ls-members $group" | awk 'NR>1 {print $5}' | paste -sd ";" -)
    
    echo "$emails"

    # Update GITHUB Secret
    SECRET_NAME="MAIL_TO_${group}"
    gh secret set "$SECRET_NAME" --body "$emails" --repo "$GITHUB_REPO"
    #gh secret set "$SECRET_NAME" --body "wclin@nuvoton.com" --repo "$GITHUB_REPO"

done

# Set default MAIL_TO
gh secret set MAIL_TO --body "wclin@nuvoton.com" --repo "$GITHUB_REPO"

# List all secrets to verify
gh secret list
