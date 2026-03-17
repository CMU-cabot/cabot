#!/bin/bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
SANDBOX_ROOT_REL="docker/home/sandbox"
SANDBOX_ROOT="${REPO_ROOT}/${SANDBOX_ROOT_REL}"
TMP_ROOT_REL="docker/home/.tmp"
SECCOMP_PROFILE_REL="cabot-navigation/docker/ros2/seccomp_profile.json"
DEBUG_SERVICE="debug"
SKIP_DOWNLOAD=0
FORCE_DOWNLOAD=0
PREPARE_ONLY=0
DEEPEN_LATEST=0
MAX_ANALYSIS_PASSES=3
ALLOW_DIRTY_TRACKED=0
WORKTREE_DIRTY=0
CODEX_BYPASS=1
MODEL=""
ISSUE_URL=""
CODEX_ARGS=()
COMPOSE_OVERRIDE_REL=""
PREVIOUS_RUN_DIR_REL=""
PREVIOUS_SUMMARY_REL=""
PREVIOUS_COMMENT_URL=""
LATEST_ISSUE_COMMENT_URL=""
CURRENT_PASS=0
FINAL_PUBLICATION_PASS=0
LAST_SUMMARY_REL=""
LAST_PR_URL=""
LAST_RUN_COMMENT_URL=""

cleanup() {
  if [[ -n "${COMPOSE_OVERRIDE_REL}" && -f "${COMPOSE_OVERRIDE_REL}" ]]; then
    rm -f "${COMPOSE_OVERRIDE_REL}"
  fi
}

trap cleanup EXIT

usage() {
  cat <<'EOF'
Usage: ./cabot_debug/script/codex_issue_debug.sh [options] <github-issue-url>

Options:
  --skip-download         Reuse an existing extracted log directory
  --force-download        Download again even if extracted logs already exist
  --prepare-only          Download logs and write prompt artifacts, but do not run Codex
  --deepen-latest         Build on the latest previous Codex run and comment for this issue
  --max-analysis-passes <n>
                          If no PR is created, keep analysis local for up to <n> passes
                          before a final publish pass (default: 3, use 0 for immediate publish)
  --debug-service <name>  Docker Compose debug service name (default: debug)
  --model <model>         Codex model passed to `codex exec`
  --codex-arg <arg>       Extra raw argument forwarded to `codex exec` (repeatable)
  --allow-dirty-tracked   Allow running with tracked top-level changes present
  --no-codex-bypass       Use `codex exec -s danger-full-access` instead of the bypass flag
  -h, --help              Show this help

Behavior:
  1. Fetches local issue snapshots with `gh issue view`.
  2. Reuses existing extracted logs under `docker/home/sandbox/<owner>-<issue>` by default; otherwise starts the debug container and runs `box_download_logs.mjs`.
  3. Writes Codex prompt artifacts under `docker/home/sandbox/<owner>-<issue>/codex-run-<timestamp>/`.
  4. Runs `codex exec` in local-analysis passes first; if no PR is created, it deepens from the saved analysis summary and only publishes on the final pass.
EOF
}

die() {
  echo "$*" >&2
  exit 1
}

require_value() {
  local flag="$1"
  local value="${2:-}"
  [[ -n "${value}" ]] || die "Missing value for ${flag}"
  echo "${value}"
}

require_nonnegative_integer() {
  local flag="$1"
  local value
  value=$(require_value "${flag}" "${2:-}")
  [[ "${value}" =~ ^[0-9]+$ ]] || die "${flag} requires a non-negative integer, got: ${value}"
  echo "${value}"
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      --skip-download)
        SKIP_DOWNLOAD=1
        shift
        ;;
      --force-download)
        FORCE_DOWNLOAD=1
        shift
        ;;
      --prepare-only)
        PREPARE_ONLY=1
        shift
        ;;
      --deepen-latest)
        DEEPEN_LATEST=1
        shift
        ;;
      --max-analysis-passes)
        MAX_ANALYSIS_PASSES=$(require_nonnegative_integer "$1" "${2:-}")
        shift 2
        ;;
      --allow-dirty-tracked)
        ALLOW_DIRTY_TRACKED=1
        shift
        ;;
      --no-codex-bypass)
        CODEX_BYPASS=0
        shift
        ;;
      --debug-service)
        DEBUG_SERVICE=$(require_value "$1" "${2:-}")
        shift 2
        ;;
      --model)
        MODEL=$(require_value "$1" "${2:-}")
        shift 2
        ;;
      --codex-arg)
        [[ $# -ge 2 ]] || die "Missing value for $1"
        CODEX_ARGS+=("$2")
        shift 2
        ;;
      -*)
        die "Unknown option: $1"
        ;;
      *)
        [[ -z "${ISSUE_URL}" ]] || die "Only one GitHub issue URL may be provided."
        ISSUE_URL="$1"
        shift
        ;;
    esac
  done

  [[ -n "${ISSUE_URL}" ]] || {
    usage >&2
    exit 1
  }

  if [[ "${SKIP_DOWNLOAD}" -eq 1 && "${FORCE_DOWNLOAD}" -eq 1 ]]; then
    die "--skip-download and --force-download cannot be used together."
  fi
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

setup_compose_override() {
  if [[ -f "${SECCOMP_PROFILE_REL}" ]]; then
    return 0
  fi

  mkdir -p "${TMP_ROOT_REL}"
  COMPOSE_OVERRIDE_REL="${TMP_ROOT_REL}/codex-debug-override-$$.yaml"

  cat > "${COMPOSE_OVERRIDE_REL}" <<'EOF'
services:
  debug:
    security_opt: !override []
EOF

  echo "Missing ${SECCOMP_PROFILE_REL}; using temporary compose override ${COMPOSE_OVERRIDE_REL}"
}

docker_compose() {
  local args=(docker compose -f docker-compose.yaml)

  if [[ -n "${COMPOSE_OVERRIDE_REL}" ]]; then
    args+=(-f "${COMPOSE_OVERRIDE_REL}")
  fi

  "${args[@]}" "$@"
}

ensure_clean_tracked() {
  local status
  status=$(git -C "${REPO_ROOT}" status --short --untracked-files=no)
  if [[ -n "${status}" ]]; then
    WORKTREE_DIRTY=1
  fi
  if [[ -n "${status}" && "${ALLOW_DIRTY_TRACKED}" -ne 1 ]]; then
    die "Tracked top-level changes are present. Commit or stash them first, or rerun with --allow-dirty-tracked."
  fi
}

parse_issue_url() {
  local url="$1"

  if [[ "${url}" =~ ^https?://([^/]+)/([^/]+)/([^/]+)/issues/([0-9]+) ]]; then
    ISSUE_HOST="${BASH_REMATCH[1]}"
    ISSUE_OWNER="${BASH_REMATCH[2]}"
    ISSUE_REPO="${BASH_REMATCH[3]}"
    ISSUE_NUMBER="${BASH_REMATCH[4]}"
    ISSUE_REPO_SLUG="${ISSUE_OWNER}/${ISSUE_REPO}"
    ISSUE_TAG="${ISSUE_OWNER}-${ISSUE_NUMBER}"
    ISSUE_URL_NORMALIZED="https://${ISSUE_HOST}/${ISSUE_OWNER}/${ISSUE_REPO}/issues/${ISSUE_NUMBER}"
    return 0
  fi

  die "Unsupported GitHub issue URL: ${url}"
}

ensure_gh_auth() {
  if [[ "${ISSUE_HOST}" == "github.com" ]]; then
    gh auth status >/dev/null
  else
    gh auth status --hostname "${ISSUE_HOST}" >/dev/null
  fi
}

timestamp_tag() {
  date -u +"%Y-%m-%dT%H-%M-%SZ"
}

find_extracted_log_entry() {
  if [[ ! -d "${ISSUE_DIR}" ]]; then
    return 0
  fi

  find "${ISSUE_DIR}" \
    -mindepth 1 \
    -maxdepth 1 \
    \( -type d -o -type f \) \
    ! -name 'codex-run-*' \
    -print | sort | head -n 1
}

fetch_issue_snapshots() {
  local issue_fields="author,body,comments,createdAt,labels,number,state,stateReason,title,updatedAt,url"

  echo "Fetching issue snapshot for ${ISSUE_URL_NORMALIZED}..."
  gh issue view \
    "${ISSUE_URL_NORMALIZED}" \
    --comments \
    --json "${issue_fields}" \
    > "${ISSUE_SNAPSHOT_JSON_REL}"

  jq -r '
    [
      "# Issue Snapshot",
      "",
      "- URL: \(.url)",
      "- Number: \(.number)",
      "- Title: \(.title)",
      "- State: \(.state)\(if (.stateReason // "") != "" then " (\(.stateReason))" else "" end)",
      "- Labels: \((.labels // []) | map(.name) | join(", ") | if . == "" then "(none)" else . end)",
      "- Created: \(.createdAt)",
      "- Updated: \(.updatedAt)",
      "",
      "## Body",
      "",
      (.body // "(empty)"),
      "",
      "## Comments",
      ""
    ] + (
      if ((.comments // []) | length) == 0 then
        ["(none)"]
      else
        (.comments // []) | map(
          [
            "### Comment by \(.author.login // "unknown")",
            "- Created: \(.createdAt // "unknown")",
            "- URL: \(.url // "(no url)")",
            "",
            (.body // "(empty)"),
            ""
          ][]
        )
      end
    ) | flatten | .[]
  ' "${ISSUE_SNAPSHOT_JSON_REL}" > "${ISSUE_SNAPSHOT_TEXT_REL}"

  LATEST_ISSUE_COMMENT_URL=$(jq -r '
    if ((.comments // []) | length) == 0 then
      ""
    else
      .comments[-1].url // ""
    end
  ' "${ISSUE_SNAPSHOT_JSON_REL}")
}

ensure_debug_container() {
  echo "Ensuring docker compose service '${DEBUG_SERVICE}' is running..."
  docker_compose --profile dev up -d "${DEBUG_SERVICE}"
}

download_logs() {
  echo "Downloading logs from Box through the debug container..."
  docker_compose exec -u developer "${DEBUG_SERVICE}" bash -lc \
    "node cabot_debug/script/box_download_logs.mjs $(printf '%q' "${ISSUE_URL_NORMALIZED}")"
}

detect_reusable_logs() {
  local existing_entry

  if [[ "${FORCE_DOWNLOAD}" -eq 1 || "${SKIP_DOWNLOAD}" -eq 1 ]]; then
    return 0
  fi

  existing_entry=$(find_extracted_log_entry)
  if [[ -n "${existing_entry}" ]]; then
    SKIP_DOWNLOAD=1
    echo "Found existing extracted logs under ${ISSUE_DIR_REL}; reusing them and skipping download."
  fi
}

find_latest_previous_run_dir() {
  if [[ ! -d "${ISSUE_DIR}" ]]; then
    return 0
  fi

  find "${ISSUE_DIR}" \
    -mindepth 1 \
    -maxdepth 1 \
    -type d \
    -name 'codex-run-*' \
    ! -name "${RUN_ID}" \
    -print | sort | while read -r dir; do
      if [[ -f "${dir}/codex_final_message.md" ]]; then
        echo "${dir}"
      fi
    done | tail -n 1
}

extract_section_url_from_summary() {
  local section_label="$1"
  local url_regex="$2"
  local summary_path="$3"

  [[ -f "${summary_path}" ]] || return 0

  awk -v section_label="${section_label}" -v url_regex="${url_regex}" '
    function trim(line) {
      sub(/^[[:space:]-]+/, "", line)
      sub(/[[:space:]]+$/, "", line)
      return line
    }
    BEGIN { capture=0 }
    {
      if ($0 ~ "^[[:space:]-]*" section_label ":") {
        line=$0
        sub("^[[:space:]-]*" section_label ":[[:space:]]*", "", line)
        if (line ~ url_regex) {
          print line
          exit
        }
        if (tolower(line) == "none") {
          exit
        }
        capture=1
        next
      }
      if (capture) {
        line=trim($0)
        if (line ~ url_regex) {
          print line
          exit
        }
        if (line == "" || tolower(line) == "none") {
          exit
        }
      }
    }
  ' "${summary_path}"
}

extract_issue_comment_url_from_summary() {
  local summary_path="$1"
  extract_section_url_from_summary "Issue comment URL" "https://github\\.com/.+#issuecomment-[0-9]+" "${summary_path}"
}

extract_pr_url_from_summary() {
  local summary_path="$1"
  extract_section_url_from_summary "PR URL" "https://github\\.com/.+/pull/[0-9]+" "${summary_path}"
}

prepare_deepen_context() {
  local previous_run_dir

  PREVIOUS_RUN_DIR_REL=""
  PREVIOUS_SUMMARY_REL=""
  PREVIOUS_COMMENT_URL=""

  if [[ "${DEEPEN_LATEST}" -ne 1 && "${CURRENT_PASS}" -le 1 ]]; then
    return 0
  fi

  previous_run_dir=$(find_latest_previous_run_dir)
  [[ -n "${previous_run_dir}" ]] || die "A deepen pass was requested, but no previous codex-run directory was found under ${ISSUE_DIR_REL}"

  PREVIOUS_RUN_DIR_REL="${previous_run_dir#${REPO_ROOT}/}"
  PREVIOUS_SUMMARY_REL="${PREVIOUS_RUN_DIR_REL}/codex_final_message.md"
  [[ -f "${PREVIOUS_SUMMARY_REL}" ]] || die "A deepen pass was requested, but ${PREVIOUS_SUMMARY_REL} does not exist."

  PREVIOUS_COMMENT_URL=$(extract_issue_comment_url_from_summary "${PREVIOUS_SUMMARY_REL}")
}

prepare_run_artifacts() {
  RUN_ID="codex-run-$(timestamp_tag)"
  RUN_DIR_REL="${ISSUE_DIR_REL}/${RUN_ID}"
  RUN_DIR="${REPO_ROOT}/${RUN_DIR_REL}"
  ISSUE_SNAPSHOT_TEXT_REL="${RUN_DIR_REL}/issue_snapshot.txt"
  ISSUE_SNAPSHOT_JSON_REL="${RUN_DIR_REL}/issue_snapshot.json"
  PROMPT_REL="${RUN_DIR_REL}/codex_prompt.txt"
  FINAL_MESSAGE_REL="${RUN_DIR_REL}/codex_final_message.md"
  LATEST_ISSUE_COMMENT_URL=""

  mkdir -p "${RUN_DIR}"
}

write_prompt() {
  local dirty_guidance=""
  local deepen_guidance=""
  local publication_guidance=""
  local output_guidance=""

  if [[ "${WORKTREE_DIRTY}" -eq 1 ]]; then
    dirty_guidance=$'- The current top-level worktree already has tracked local changes. Do not create commits or PRs in this run; limit yourself to analysis and GitHub issue comments.\n'
  fi

  if [[ -n "${PREVIOUS_SUMMARY_REL}" ]]; then
    deepen_guidance="- This is a follow-up deep-dive run. Read the previous run summary at \`${PREVIOUS_SUMMARY_REL}\` before analyzing the issue again."$'\n'
    if [[ -n "${PREVIOUS_COMMENT_URL}" ]]; then
      deepen_guidance+="- Build on the previous Codex issue comment at \`${PREVIOUS_COMMENT_URL}\`; do not just restate it."$'\n'
    fi
    if [[ -n "${LATEST_ISSUE_COMMENT_URL}" ]]; then
      deepen_guidance+="- The latest issue comment in the current snapshot is \`${LATEST_ISSUE_COMMENT_URL}\`; use it as the current public baseline."$'\n'
    fi
    deepen_guidance+=$'- Focus on why the behavior happens by tying actual log lines, bag contents, and the relevant code together. Prefer a causal explanation over a generic checklist.\n'
    deepen_guidance+=$'- Try to confirm or reject at least one concrete mechanism from the previous analysis. If you still cannot fully prove it, explain the strongest mechanism, the competing mechanism, and the specific evidence for each.\n'
    deepen_guidance+=$'- If you post a follow-up GitHub comment, include only new or sharpened findings beyond the previous comment, and keep the required Codex disclosure on the first line.\n'
  fi

  if [[ "${FINAL_PUBLICATION_PASS}" -eq 1 ]]; then
    publication_guidance="- This is the final publication pass."$'\n'
    if [[ "${MAX_ANALYSIS_PASSES}" -gt 0 ]]; then
      publication_guidance+="- The previous ${MAX_ANALYSIS_PASSES} pass(es) were local-only analysis passes. Use their saved summaries to sharpen the final public output.\n"
    fi
    publication_guidance+=$'- In this pass, if you cannot justify a PR, you must post the best GitHub issue comment you can support with evidence.\n'
    output_guidance=$'- In your final summary, include whether this final pass published a PR or an issue comment.\n'
  else
    publication_guidance="- This is local analysis pass ${CURRENT_PASS} of ${MAX_ANALYSIS_PASSES} before any public fallback comment."$'\n'
    publication_guidance+=$'- You may create a PR immediately if you have a concrete, sufficiently justified patch and machine-checkable validation.\n'
    publication_guidance+=$'- If you do not create a PR in this pass, do not post a GitHub issue comment. Keep the issue untouched and leave the analysis only in local artifacts.\n'
    output_guidance=$'- If no PR is created in this pass, use your final summary to capture the strongest mechanism, at least one rejected alternative, candidate patch points or instrumentation hooks, and the next evidence a later deepen pass should check.\n'
  fi

  cat > "${PROMPT_REL}" <<EOF
Handle GitHub issue ${ISSUE_URL_NORMALIZED} from the repository root.

Mandatory workflow:
- Read \`AGENTS.md\` and \`AGENTS-DEBUG.md\` before deciding anything.
- Use the downloaded logs under \`${ISSUE_DIR_REL}\`.
- Use the local issue snapshots at \`${ISSUE_SNAPSHOT_TEXT_REL}\` and \`${ISSUE_SNAPSHOT_JSON_REL}\` as a starting point, then use \`gh\` if you need fresher issue state.
- Follow the \`Issue Analysis Rules\` and \`Codex Output And Triage\` rules from \`AGENTS-DEBUG.md\`.
- If \`host_ws/install/setup.bash\` exists, source it before using \`ros2\` CLI tools or repo-local ROS bag analysis helpers.
- Prefer \`ros2 bag info\`, topic inspection, and repo-local helpers under \`cabot_debug/src\`, \`cabot_debug/*.sh\`, or \`cabot_debug/launch\` over relying only on plain-text log greps when bag data exists.
- When you claim a runtime mechanism, tie it to specific bag files, topics, message timing, text logs, and the relevant code path. If bag inspection is not possible, say exactly why.
- Classify the issue as either \`codex_pr_ok\` or \`human_validation_required\`.
- In GitHub-visible text, do not include private Box URLs or paths, operator names, or venue-specific internal notes.
- Do not touch unrelated untracked files.
${dirty_guidance}
${deepen_guidance}
${publication_guidance}

Decision policy:
- Default to code-level analysis when logs, bags, config, or source are available, even if the final output is only a GitHub comment.
- If the issue is \`human_validation_required\`, treat that as a validation label, not as a blanket ban on code changes or PRs. You may still prepare a narrow PR when there is a concrete candidate fix, instrumentation improvement, or guardrail change, but you must clearly separate what was validated automatically from what still needs human or robot verification.
- For \`human_validation_required\`, avoid speculative wide runtime-behavior tuning when the evidence chain is weak. If no specific patch is justified yet, keep building the best evidence-backed explanation you can, and on a publication pass use a concise GitHub issue comment that summarizes the likely symptom, concrete log evidence, the relevant code path, and the first logs/topics/config to inspect next.
- If the issue is \`codex_pr_ok\`, implement the smallest viable fix, verify it with machine-checkable commands, commit with sign-off, push a branch, create a PR, and comment on the issue with the PR link and validation summary.
- If the repository state or evidence is insufficient for a safe PR, prefer a GitHub issue comment over a speculative code change.

Git requirements:
- Follow \`AGENTS.md\` for nested repositories and commit ordering.
- Every commit must be signed off with \`git commit -s\`.
- If you touch a nested repository, inspect and commit there first, then update the top-level repository if needed.

Output requirements:
- Your final human-readable summary will be captured at \`${FINAL_MESSAGE_REL}\`.
- In that summary, include: classification, actions taken, validation commands, issue comment URL if posted, and PR URL if created.
- If no issue comment or PR is created, say so explicitly.
- The summary should be useful as the starting point for the next deepen pass without rereading everything from scratch.
- Record any concrete patch points you considered, even if you chose not to patch yet.
- Record which alternative mechanisms you ruled out and why.
- Record which bag files, topics, logs, and code paths carried the most evidence.
- If you create a PR, include the verification boundary and what still needs human validation, if anything.
- If you create an issue comment, keep the Codex disclosure on the first line.
${output_guidance}
- Also print the same summary as your final response.

Artifacts for this run may be stored under \`${RUN_DIR_REL}\`.
EOF
}

run_codex() {
  local codex_cmd=(codex exec)

  if [[ "${CODEX_BYPASS}" -eq 1 ]]; then
    codex_cmd+=(--dangerously-bypass-approvals-and-sandbox)
  else
    codex_cmd+=(-s danger-full-access)
  fi

  if [[ -n "${MODEL}" ]]; then
    codex_cmd+=(-m "${MODEL}")
  fi

  codex_cmd+=(-o "${FINAL_MESSAGE_REL}")
  codex_cmd+=("${CODEX_ARGS[@]}")
  codex_cmd+=(-)

  echo "Running Codex: ${codex_cmd[*]}"
  "${codex_cmd[@]}" < "${PROMPT_REL}"
}

run_codex_passes() {
  local total_passes
  local pass_label

  total_passes=$((MAX_ANALYSIS_PASSES + 1))

  for ((CURRENT_PASS=1; CURRENT_PASS<=total_passes; CURRENT_PASS++)); do
    if [[ "${CURRENT_PASS}" -eq "${total_passes}" ]]; then
      FINAL_PUBLICATION_PASS=1
      pass_label="final publication pass"
    else
      FINAL_PUBLICATION_PASS=0
      pass_label="analysis-only pass ${CURRENT_PASS}/${MAX_ANALYSIS_PASSES}"
    fi

    echo "Preparing ${pass_label}..."
    prepare_run_artifacts
    fetch_issue_snapshots
    prepare_deepen_context
    write_prompt
    echo "Issue artifacts written to ${RUN_DIR_REL}"

    if [[ "${PREPARE_ONLY}" -eq 1 ]]; then
      echo "Preparation complete. Codex execution was skipped by --prepare-only."
      exit 0
    fi

    run_codex
    LAST_SUMMARY_REL="${FINAL_MESSAGE_REL}"
    LAST_PR_URL=$(extract_pr_url_from_summary "${FINAL_MESSAGE_REL}")
    LAST_RUN_COMMENT_URL=$(extract_issue_comment_url_from_summary "${FINAL_MESSAGE_REL}")

    if [[ -n "${LAST_PR_URL}" ]]; then
      echo "PR created during ${pass_label}: ${LAST_PR_URL}"
      return 0
    fi

    if [[ "${FINAL_PUBLICATION_PASS}" -eq 0 ]]; then
      if [[ -n "${LAST_RUN_COMMENT_URL}" ]]; then
        die "Codex posted an issue comment during an analysis-only pass (${CURRENT_PASS}). See ${LAST_RUN_COMMENT_URL}"
      fi
      echo "No PR created during ${pass_label}; keeping local analysis summary and continuing to the next deepen pass."
      continue
    fi

    if [[ -n "${LAST_RUN_COMMENT_URL}" ]]; then
      echo "Issue comment created during ${pass_label}: ${LAST_RUN_COMMENT_URL}"
      return 0
    fi

    die "Final publication pass completed without creating a PR or GitHub issue comment. See ${FINAL_MESSAGE_REL}"
  done
}

main() {
  parse_args "$@"
  parse_issue_url "${ISSUE_URL}"

  require_command git
  require_command gh
  require_command docker
  require_command codex
  require_command jq

  cd "${REPO_ROOT}"

  ensure_clean_tracked
  ensure_gh_auth
  setup_compose_override

  ISSUE_DIR_REL="${SANDBOX_ROOT_REL}/${ISSUE_TAG}"
  ISSUE_DIR="${SANDBOX_ROOT}/${ISSUE_TAG}"
  mkdir -p "${ISSUE_DIR}"

  detect_reusable_logs

  if [[ "${SKIP_DOWNLOAD}" -ne 1 ]]; then
    ensure_debug_container
    download_logs
  else
    echo "Skipping log download and reusing the existing sandbox directory."
  fi

  local downloaded_entry
  downloaded_entry=$(find_extracted_log_entry)
  [[ -n "${downloaded_entry}" ]] || die "No downloaded log artifacts were found under ${ISSUE_DIR_REL}"

  run_codex_passes
  echo "Codex summary saved to ${LAST_SUMMARY_REL}"
}

main "$@"
