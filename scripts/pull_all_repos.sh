#!/usr/bin/env bash
set -u

#
# put this script in the source root of your workspace (~/robot_ws/src)
# and run it to update all git repositories in the sub-directories
#
# The script lists the sub-directories, switch to each of them and performs a git pull there
#

BASE_DIR="${1:-$PWD}"

for dir in "$BASE_DIR"/*/; do

    if [ ! -d "$dir/.git" ]; then
        echo
        echo "*** Skipping $dir (not a git repository) ***"
        continue
    fi

    echo
    echo "=== Updating $(basename "$dir") ==="

    (
        cd "$dir" || exit 1
        git status | grep "On branch"
        git remote show origin | grep "Fetch URL"
	git status | grep "modified:"
        git pull
    )
done
