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
    local pattern="$1"
    local exclude="${2:-}"

    list_matching_files "${pattern}" "${exclude}" | wc -l
}

matching_snapshot() {
    local pattern="$1"
    local exclude="${2:-}"
    local file=

    while IFS= read -r file; do
        [[ -n "${file}" ]] || continue
        stat -c '%n:%s' "${file}"
    done < <(list_matching_files "${pattern}" "${exclude}") | sort
}

list_matching_files() {
    local pattern="$1"
    local exclude="${2:-}"
    local find_args=(. -maxdepth 1 -type f -name "${pattern}")

    if [[ -n "${exclude}" ]]; then
        find_args+=("!" -name "${exclude}")
    fi

    find "${find_args[@]}" -printf '%P\n' | sort
}

wait_for_stable_exact_file() {
    local file="$1"
    local first=
    local second=

    until [[ -e "${file}" ]]; do
        echo "Waiting for ${file} file"
        sleep 15
    done

    while true; do
        first=$(stat -c '%s' "${file}" 2>/dev/null || true)
        sleep 5
        second=$(stat -c '%s' "${file}" 2>/dev/null || true)
        if [[ -n "${first}" && "${first}" == "${second}" ]]; then
            return 0
        fi
        echo "Waiting for ${file} download to finish"
        sleep 10
    done
}

wait_for_stable_file_set() {
    local pattern="$1"
    local expected="$2"
    local exclude="${3:-}"
    local count=0
    local first=
    local second=

    while true; do
        count=$(count_matching_files "${pattern}" "${exclude}")
        if [[ "${count}" -ne "${expected}" ]]; then
            echo "Waiting for ${count}/${expected} files matching ${pattern}"
            sleep 15
            continue
        fi

        first=$(matching_snapshot "${pattern}" "${exclude}")
        sleep 5
        second=$(matching_snapshot "${pattern}" "${exclude}")
        if [[ -n "${first}" && "${first}" == "${second}" ]]; then
            return 0
        fi
        echo "Waiting for ${count}/${expected} files matching ${pattern} to finish downloading"
        sleep 10
    done
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
    wait_for_stable_exact_file "${prefix}_log.tar"
    tar xf "${prefix}_log.tar" -C "$output_dir"
    rm -f "${prefix}_log.tar"
fi

if [[ $num_files -eq 0 ]]; then
    exit 0
fi

base=${prefix}
if [[ $separated -eq 1 ]]; then
    base=${prefix}_ros2_topics
fi

if [[ $num_files -eq 1 ]]; then
    wait_for_stable_exact_file "${base}.tar"
    tar xf "${base}.tar" -C "$output_dir"
    rm -f "${base}.tar"
    exit 0
fi

pattern="${base}*"
if [[ $separated -eq 1 ]]; then
    pattern="${base}_part_*"
fi

wait_for_stable_file_set "${pattern}" "${num_files}" "${base}.tar"

mapfile -t matching_files < <(list_matching_files "${pattern}" "${base}.tar")
cat "${matching_files[@]}" | tar -xf - -C "$output_dir"
rm -f "${matching_files[@]}"

echo "Files have been combined and extracted to $output_dir"
