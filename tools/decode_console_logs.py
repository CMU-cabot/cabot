#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Iterable

ENTRY_RE = re.compile(r"^(?:(?P<source>.+?)\s+\|\s+)?(?P<level>[A-Z]+):\s+(?P<message>.*)$")
LOG_RE = re.compile(r"(?P<key>LOG_[A-Z0-9\[\]_]+):\s*(?P<payload>.*)$")
PIPE_PREFIX_RE = re.compile(r"^(?P<source>.+?)\s+\|\s?(?P<rest>.*)$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode local console logs that contain LOG_* JSON payloads."
    )
    parser.add_argument(
        "src",
        nargs="?",
        default="convasation.log",
        help="source console log file",
    )
    parser.add_argument(
        "dst",
        nargs="?",
        default="decoded_console.json",
        help="output JSON file",
    )
    return parser.parse_args()


def strip_container_prefix(line: str, source: str | None) -> str:
    if not source:
        return line
    match = PIPE_PREFIX_RE.match(line)
    if match and match.group("source") == source:
        return match.group("rest")
    return line


def finalize_entry(entry: dict[str, object]) -> dict[str, object]:
    payload_lines = entry.pop("payload_lines")
    assert isinstance(payload_lines, list)
    payload_text = "\n".join(str(line) for line in payload_lines).strip()

    try:
        data = json.loads(payload_text)
    except json.JSONDecodeError as exc:
        data = {
            "error": str(exc),
            "raw": payload_text,
        }

    conversation_id = data.get("conversation_id") if isinstance(data, dict) else None
    item = {
        "timestamp": None,
        "line_number": entry["line_number"],
        "source": entry["source"],
        "level": entry["level"],
        "key": entry["key"],
        "conversation_id": conversation_id,
        "data": data,
    }
    return item


def parse_console_log(lines: Iterable[str]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    current: dict[str, object] | None = None

    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.rstrip("\n")
        entry_match = ENTRY_RE.match(line)

        if entry_match:
            log_match = LOG_RE.search(entry_match.group("message"))
            if log_match:
                if current is not None:
                    out.append(finalize_entry(current))
                current = {
                    "line_number": line_number,
                    "source": entry_match.group("source"),
                    "level": entry_match.group("level"),
                    "key": log_match.group("key"),
                    "payload_lines": [log_match.group("payload")],
                }
                continue
            if current is not None:
                out.append(finalize_entry(current))
                current = None

        if current is None:
            continue

        current_source = current.get("source")
        current["payload_lines"].append(
            strip_container_prefix(line, current_source if isinstance(current_source, str) else None)
        )

    if current is not None:
        out.append(finalize_entry(current))

    return out


def build_suitcase_map(rows: list[dict[str, object]]) -> dict[str, str]:
    suitcase_by_conversation: dict[str, str] = {}
    for row in rows:
        if row.get("key") != "LOG_API_REQUEST":
            continue
        data = row.get("data")
        conversation_id = row.get("conversation_id")
        if not isinstance(data, dict) or not isinstance(conversation_id, str):
            continue
        suitcase_id = data.get("suitcase_id")
        if isinstance(suitcase_id, str) and suitcase_id:
            suitcase_by_conversation[conversation_id] = suitcase_id
    return suitcase_by_conversation


def print_header(conversation_id: str | None, suitcase_by_conversation: dict[str, str]) -> None:
    if conversation_id:
        suitcase_id = suitcase_by_conversation.get(conversation_id, "Unknown")
        print(f"\n========== {suitcase_id} / {conversation_id[:8]} ==========")
        return
    print("\n========== Unknown / no-conversation ==========")


def format_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def nested_dict(value: object, key: str) -> dict[str, object]:
    if isinstance(value, dict):
        child = value.get(key)
        if isinstance(child, dict):
            return child
    return {}


def print_summary(rows: list[dict[str, object]]) -> None:
    suitcase_by_conversation = build_suitcase_map(rows)
    active_conversation_id: str | None = None

    for row in rows:
        key = row["key"]
        data = row["data"]
        line_number = row["line_number"]
        conversation_id = row["conversation_id"]

        needs_header = conversation_id != active_conversation_id
        if key == "LOG_API_REQUEST":
            if needs_header:
                print_header(conversation_id if isinstance(conversation_id, str) else None, suitcase_by_conversation)
                active_conversation_id = conversation_id if isinstance(conversation_id, str) else None
            continue

        if key == "LOG_USER_INPUT" and isinstance(data, dict):
            if needs_header:
                print_header(conversation_id if isinstance(conversation_id, str) else None, suitcase_by_conversation)
                active_conversation_id = conversation_id if isinstance(conversation_id, str) else None
            print(f"L{line_number} User: {data.get('text', '')}")
        elif key == "LOG_ASSISTANT_OUTPUT" and isinstance(data, dict):
            if needs_header:
                print_header(conversation_id if isinstance(conversation_id, str) else None, suitcase_by_conversation)
                active_conversation_id = conversation_id if isinstance(conversation_id, str) else None
            print(f"L{line_number}   AI: {data.get('text', '')}")
        elif key == "LOG_TOOLCALL_INPUT" and isinstance(data, dict):
            if needs_header:
                print_header(conversation_id if isinstance(conversation_id, str) else None, suitcase_by_conversation)
                active_conversation_id = conversation_id if isinstance(conversation_id, str) else None
            tool_name = data.get("name")
            tool_args = data.get("args")
            if isinstance(tool_name, str):
                print(f"L{line_number} Tool: {tool_name}({format_json(tool_args)})")
            else:
                print(f"L{line_number} Tool: {format_json(data)}")
        elif key == "LOG_LLM_OUTPUT[0]" and isinstance(data, dict):
            if needs_header:
                print_header(conversation_id if isinstance(conversation_id, str) else None, suitcase_by_conversation)
                active_conversation_id = conversation_id if isinstance(conversation_id, str) else None
            token_usage = nested_dict(nested_dict(data, "response_metadata"), "token_usage")
            if isinstance(token_usage, dict):
                total_tokens = token_usage.get("total_tokens")
                cached_tokens = nested_dict(token_usage, "prompt_tokens_details").get("cached_tokens")
                if isinstance(total_tokens, int) and isinstance(cached_tokens, int):
                    print(
                        f"L{line_number}  LLM: {total_tokens} tokens - (cached: {cached_tokens}) = {total_tokens - cached_tokens} tokens"
                    )


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    src = Path(args.src)
    dst = Path(args.dst)

    if not src.is_absolute() and not src.exists():
        src = script_dir / src
    if not dst.is_absolute() and args.dst == "decoded_console.json":
        dst = script_dir / dst

    rows = parse_console_log(src.read_text(encoding="utf-8").splitlines())
    print_summary(rows)
    dst.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
