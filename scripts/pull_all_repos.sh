#!/usr/bin/env bash

# Update each Git repository directly below a workspace source directory.
#
# put this script in the source root of your workspace (~/robot_ws/src)
# and run it to update all git repositories in the sub-directories:
#
#     cd ~/robot_ws/src
#     ./pull_all_repos.sh
#
# or, from the workspace root, run:
#
#     bash src/articubot_one/scripts/pull_all_repos.sh src
#
# The optional directory argument defaults to the current directory. Expect
# a progress heading and Git status details for each repository, followed by
# its git pull output. The script prints a skip message for other directories.

set -u

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
