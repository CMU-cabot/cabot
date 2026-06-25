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

if [ "$(id -u)" -eq 0 ]; then
   echo "please do not run as root: $0"
   exit 1
fi

install_dir="$HOME/.config/systemd/user/cabot.service.d"
dropin_file="$install_dir/wait-ethlan5.conf"
wait_script="/opt/cabot/tools/wait-network-interface.sh"
interface="ethLAN5"
timeout=30
address=""
command=""

function red {
    echo -en "\033[31m"
    echo "$@"
    echo -en "\033[0m"
}

function blue {
    echo -en "\033[36m"
    echo "$@"
    echo -en "\033[0m"
}

function help() {
    echo "Usage:"
    echo "  $0 <command> [-i interface] [-t timeout_sec] [-a ipv4_address]"
    echo ""
    echo "Commands:"
    echo "  -c          Check the current drop-in."
    echo "  -h          Show this help message."
    echo "  -i          Install/update the drop-in."
    echo "  -u          Uninstall the drop-in."
    echo ""
    echo "Options:"
    echo "  -n IFACE    Network interface to wait for. Default: ${interface}"
    echo "  -t SEC      Timeout in seconds. Default: ${timeout}"
    echo "  -a ADDR     Required IPv4 address. If omitted, any global IPv4 is accepted."
}

function check_command() {
    if [[ -n $command ]]; then
        red "Cannot specify multiple commands."
        help
        exit 1
    fi
}

while getopts "hciun:t:a:" arg; do
    case $arg in
        c)
            check_command
            command="check"
            ;;
        h)
            help
            exit 0
            ;;
        i)
            check_command
            command="install"
            ;;
        u)
            check_command
            command="uninstall"
            ;;
        n)
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

if [[ -z $command ]]; then
    red "No command selected."
    help
    exit 1
fi

if ! [[ "$timeout" =~ ^[0-9]+$ ]]; then
    red "timeout must be a non-negative integer: ${timeout}"
    exit 1
fi

function print_dropin() {
    local args="-i ${interface} -t ${timeout}"
    if [[ -n "$address" ]]; then
        args="${args} -a ${address}"
    fi

    cat <<EOF
[Unit]
Wants=network-online.target
After=network-online.target NetworkManager.service NetworkManager-wait-online.service

[Service]
ExecStartPre=${wait_script} ${args}
EOF
}

function check() {
    blue "- wait script"
    if [[ -x "$wait_script" ]]; then
        blue "  - found: $wait_script"
    else
        red "  - missing or not executable: $wait_script"
    fi

    blue "- drop-in"
    if [[ -f "$dropin_file" ]]; then
        blue "  - installed: $dropin_file"
        sed 's/^/    /' "$dropin_file"
    else
        red "  - not installed: $dropin_file"
    fi
}

function install_dropin() {
    if [[ ! -x "$wait_script" ]]; then
        red "wait helper is missing or not executable: $wait_script"
        red "install /opt/cabot/tools/wait-network-interface.sh first"
        exit 1
    fi

    blue "- Installing drop-in"
    mkdir -p "$install_dir"
    print_dropin > "$dropin_file"
    systemctl --user daemon-reload
    blue "- Installed: $dropin_file"
}

function uninstall_dropin() {
    blue "- Uninstalling drop-in"
    rm -f "$dropin_file"
    systemctl --user daemon-reload
    blue "- Removed: $dropin_file"
}

case "$command" in
    check)
        check
        ;;
    install)
        install_dropin
        ;;
    uninstall)
        uninstall_dropin
        ;;
esac
