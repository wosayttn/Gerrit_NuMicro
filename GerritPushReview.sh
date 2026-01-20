#!/bin/bash

set -e

curl -Lo .git/hooks/commit-msg http://10.1.8.206/tools/hooks/commit-msg

chmod +x .git/hooks/commit-msg

git commit --amend --no-edit

git push origin HEAD:refs/for/master
