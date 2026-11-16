#!/usr/bin/env bash
#
# Show the current branch and modified files for each Git repository
# directly below a directory.
#
# From the workspace root, run:
#
#     bash src/articubot_one/scripts/show_branches.sh src
#
# The directory defaults to the current directory. Expect a heading for
# each repository, its branch when checked out, the origin fetch URL, and
# any modified-file lines from Git status. The origin lookup can contact
# the remote. Directories without a .git directory are skipped.

set -u

BASE_DIR="${1:-$PWD}"

for dir in "$BASE_DIR"/*/; do

    if [ ! -d "$dir/.git" ]; then
        echo
        echo "*** Skipping $dir (not a git repository) ***"
        continue
    fi

    echo
    echo "=== Status of $(basename "$dir") ==="

    (
        cd "$dir" || exit 1

        git status | grep "On branch"
        git remote show origin | grep "Fetch URL"
	git status | grep "modified:"
    )
done
