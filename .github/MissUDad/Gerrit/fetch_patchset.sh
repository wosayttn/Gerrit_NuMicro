#!/bin/sh

SERVER="http://10.1.8.206"
LOGIN="wclin:bgt5nHY^"

if [ -z "$1" ]; then
    echo "Missing Gerrit Change Number"
    echo "Usage: $0 <Change Number>"
    exit 1
fi

get_max_revision() {
    local change_number="$1"

    REV=$(curl -s -u "$LOGIN" "$SERVER/a/changes/$change_number/detail" \
        | grep -v ")]}'" \
        | ./jq '[.messages[]._revision_number] | max')

    [[ -z "$REV" || "$REV" == "null" ]] && return 0
    echo "$REV"
}

set -e
set -x

change_number="$1"
current_patchset="$(get_max_revision "$change_number")"

if [ -z "$current_patchset" ]; then
    echo "Could not find patchset for change $change_number"
    exit 1
fi

# Gerrit ref needs the last two digits of the change number
last_two=$(printf "%02d" $((change_number % 100)))

ref="refs/changes/$last_two/$change_number/$current_patchset"

echo "Fetching $ref ..."

git fetch origin "$ref"
git checkout -b "change_${change_number}_${current_patchset}" FETCH_HEAD
