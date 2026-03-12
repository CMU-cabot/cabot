#!/bin/bash

set -euo pipefail

help() {
    echo "Usage: $0 -p <prefix> [option]"
    echo ""
    echo "-s             separated"
    echo "-p <prefix>    prefix to watch"
    echo "-n <num>       num of divided files, 1 if not specified"
    echo "-o <dir>       output dir, ./ if not specified"
    echo "-i <name>      issue name"
}

count_matching_files() {
    local base="$1"
    find . -maxdepth 1 -type f -name "${base}*" | wc -l
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)

output_dir=./
num_files=1
issue_dir="${REPO_ROOT}/docker/home/sandbox"
separated=0
issue=

while getopts "hp:n:o:i:s" opt; do
  case $opt in
    h) help; exit ;;
    p) prefix="$OPTARG" ;;
    n) num_files="$OPTARG" ;;
    o) output_dir="$OPTARG" ;;
    i) issue="$OPTARG" ;;
    s) separated=1 ;;
    *) echo "Invalid option: -$OPTARG" >&2; exit 1 ;;
  esac
done

if [[ -z "${prefix:-}" ]]; then
    help
    exit 1
fi

if [[ -n "${issue}" ]]; then
  output_dir="${issue_dir}/${issue}"
fi

echo "making dir $output_dir"
mkdir -p "$output_dir"

if [[ $separated -eq 1 ]]; then
    until [[ -e ${prefix}_log.tar ]]; do
        echo "Waiting for ${prefix}_log.tar file"
        sleep 15
    done
    tar xfv "${prefix}_log.tar" -C "$output_dir"
fi

if [[ $num_files -eq 0 ]]; then
    exit 0
fi

base=${prefix}
if [[ $separated -eq 1 ]]; then
    base=${prefix}_ros2_topics
fi

count=$(count_matching_files "${base}")
until [[ $count -eq $num_files ]]; do
    echo "Waiting for ${count}/${num_files} files having prefix ${base}"
    sleep 15
    count=$(count_matching_files "${base}")
done

if [[ $num_files -eq 1 ]]; then
    tar xfv "${base}.tar" -C "$output_dir"
    exit 0
fi

cat "${base}"* > "${base}.tar"
tar -xf "${base}.tar" -C "$output_dir"
rm "${base}.tar"

echo "Files have been combined and extracted to $output_dir"
