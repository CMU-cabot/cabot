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

interface="ethLAN5"
timeout=30
address=""

function help() {
    echo "Usage:"
    echo "  $0 [-i interface] [-t timeout_sec] [-a ipv4_address]"
    echo ""
    echo "Options:"
    echo "  -i    Network interface to wait for. Default: ${interface}"
    echo "  -t    Timeout in seconds. Default: ${timeout}"
    echo "  -a    Required IPv4 address. If omitted, any global IPv4 address is accepted."
    echo "  -h    Show this help message."
}

while getopts "hi:t:a:" arg; do
    case $arg in
        h)
            help
            exit 0
            ;;
        i)
            interface="$OPTARG"
            ;;
        t)
            timeout="$OPTARG"
            ;;
        a)
            address="$OPTARG"
            ;;
        *)
            help
            exit 1
            ;;
    esac
done

if ! command -v ip >/dev/null 2>&1; then
    echo "[ERROR] ip command is not available" >&2
    exit 1
fi

if ! [[ "$timeout" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] timeout must be a non-negative integer: ${timeout}" >&2
    exit 1
fi

function has_global_ipv4() {
    ip -4 -o addr show dev "$interface" scope global | grep -q "inet "
}

function has_required_ipv4() {
    ip -4 -o addr show dev "$interface" scope global | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "$address"
}

deadline=$((SECONDS + timeout))

while ((SECONDS <= deadline)); do
    if [[ ! -d "/sys/class/net/${interface}" ]]; then
        sleep 1
        continue
    fi

    carrier=$(cat "/sys/class/net/${interface}/carrier" 2>/dev/null || echo 0)
    if [[ "$carrier" != "1" ]]; then
        sleep 1
        continue
    fi

    if [[ -n "$address" ]]; then
        if has_required_ipv4; then
            echo "${interface} is ready with IPv4 address ${address}"
            exit 0
        fi
    else
        if has_global_ipv4; then
            echo "${interface} is ready with a global IPv4 address"
            exit 0
        fi
    fi

    sleep 1
done

echo "[ERROR] Timed out after ${timeout}s waiting for ${interface}" >&2
if [[ -d "/sys/class/net/${interface}" ]]; then
    echo "[ERROR] Current link state:" >&2
    ip link show dev "$interface" >&2 || true
    echo "[ERROR] Current IPv4 addresses:" >&2
    ip -4 addr show dev "$interface" >&2 || true
else
    echo "[ERROR] Interface ${interface} does not exist" >&2
fi
exit 1
