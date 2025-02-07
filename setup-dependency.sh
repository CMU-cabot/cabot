#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}


function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-c                    clean (rm -rf) dependency repositories"
    echo "-n <count>            max count for recursive check (default=3)"
    echo "-r                    update depedency-release.repos"
    echo "-d                    use dependency.repos and dependency-dev.repos not dependency-release.repos"
    echo "-o                    use dependency-override.repos"
    echo "-s                    **DEPRECATED** use -o option instead"
    echo ""
    echo "# dependency.repos          # refer sub repos by mainly branches, cyclic"
    echo "# dependency-override.repos # override dependency.repos for dev branches (you can commit)"
    echo "# dependency-release.repose # freezed one, including all sub-sub repos, non-cyclic"
    echo ""
    echo "example"
    echo "$0                  # most common"
    echo "$0 -d               # for development"
    echo ""
    echo "edit dependency-override.repos  # override reuqired repos branches"
    echo "$0 -d -o            # use dependency.repos overwritten by dependency-override.repos"
}

clean=0
count=3
release=0
development=0
override=0

while getopts "hcdn:or" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        c)
            clean=1
            ;;
	d)
	    development=1
	    ;;
        n)
            count=$OPTARG
            ;;
	o)
	    override=1
	    ;;
        r)
            release=1
            ;;
	*)
	    help
	    exit
	    ;;
    esac
done

## export dependencies to dependency-release.repos
if [[ $release -eq 1 ]]; then
    mv .git .git-back  # work around to eliminate the current repository itself
    vcs export -n --exact-with-tags > dependency-release.repos
    mv .git-back .git  # restore the .git dir
    exit
fi


if [[ $clean -eq 1 ]]; then
    pwd=$(pwd)
    find * -name ".git" | while read -r line; do
        if [[ -d $line/../ ]]; then
            pushd $line/../
            if git diff --quiet && ! git ls-files --others --exclude-standard | grep -q .; then
                echo "rm -rf $pwd/$(dirname $line)"
                rm -rf $pwd/$(dirname $line)
            else
                blue "There are unstaged/untracked changes in $line"
            fi
            popd
        fi
    done
    exit
fi


## for release
if [[ -e dependency-release.repos ]] && [[ $development -eq 0 ]]; then
    echo "setup dependency from release"
    vcs import < dependency-release.repos
    exit
fi


## for dev
declare -A visited

for (( i=1; i<=count; i++ ))
do
    files=$(find . -name "dependency.repos" | awk '{print length, $0}' | sort -n | cut -d' ' -f2-)

    flag=0
    for line in ${files[@]}; do
        if [[ -z ${visited[$line]} ]]; then
            flag=1
            visited[$line]=1

            temp_file=$(mktemp)
            echo "Temporary file created: $temp_file"
	    if [[ $override -eq 1 ]]; then
		cat $line > $temp_file
		cat ${line/.repos/-override.repos} | sed s/repositories:// >> $temp_file
  	    else
		cat $line > $temp_file
	    fi
            blue "$(dirname $line)/ vcs import < $temp_file"
            pushd $(dirname $line)
            vcs import < $temp_file
            popd
        fi
    done
    if [[ $flag -eq 0 ]]; then
        break
    fi
done
