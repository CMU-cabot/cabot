#!/bin/bash

# Copyright (c) 2026  Carnegie Mellon University
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

set -u

env_file="/opt/cabot/.env"
ssid_timeout="${CABOT_ROTATE_SCREEN_WAIT_TIMEOUT:-10}"
display_timeout="${CABOT_ROTATE_SCREEN_DISPLAY_WAIT_TIMEOUT:-10}"

if [[ -f "$env_file" ]]; then
    set -a
    source "$env_file"
    set +a
fi

target_ssid="${CABOT_LANDSCAPE_SSID:-}"

if ! [[ "$ssid_timeout" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] invalid CABOT_ROTATE_SCREEN_WAIT_TIMEOUT: ${ssid_timeout}" >&2
    exit 1
fi

if ! [[ "$display_timeout" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] invalid CABOT_ROTATE_SCREEN_DISPLAY_WAIT_TIMEOUT: ${display_timeout}" >&2
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

function current_ssid() {
    if command -v iwgetid >/dev/null 2>&1; then
        iwgetid -r
        return
    fi

    return 1
}

deadline=$((SECONDS + ssid_timeout))
ssid=""

while ((SECONDS <= deadline)); do
    ssid="$(current_ssid 2>/dev/null || true)"
    if [[ -n "$ssid" ]]; then
        break
    fi
    sleep 1
done

rotate="left"
if [[ -n "$target_ssid" && "$ssid" == "$target_ssid" ]]; then
    rotate="normal"
fi

deadline=$((SECONDS + display_timeout))
display=""

while ((SECONDS <= deadline)); do
    display="$(xrandr 2>/dev/null | grep " connected" | cut -d " " -f 1 | head -n 1)"
    if [[ -n "$display" ]]; then
        break
    fi
    sleep 1
done

if [[ -z "$display" ]]; then
    echo "[ERROR] no connected display found" >&2
    exit 1
fi

xrandr --output "$display" --rotate "$rotate"
