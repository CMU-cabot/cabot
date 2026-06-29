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

scriptdir="$(cd "$(dirname "$0")" && pwd)"
service_name="rotate-screen-by-ssid.service"
template_file="$scriptdir/config/$service_name"
install_dir="$HOME/.config/systemd/user"
install_file="$install_dir/$service_name"
command=""

function help() {
    echo "Usage:"
    echo "  $0 <command>"
    echo ""
    echo "Commands:"
    echo "  -h          Show this help message."
    echo "  -i          Install and enable the service."
    echo "  -u          Disable and uninstall the service."
}

function check_command() {
    if [[ -n $command ]]; then
        echo "Cannot specify multiple commands." >&2
        help
        exit 1
    fi
}

while getopts "hiu" arg; do
    case $arg in
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
        *)
            help
            exit 1
            ;;
    esac
done

if [[ -z $command ]]; then
    echo "No command selected." >&2
    help
    exit 1
fi

function install_service() {
    if [[ ! -f "$template_file" ]]; then
        echo "Service template not found: $template_file" >&2
        exit 1
    fi

    mkdir -p "$install_dir"
    cp "$template_file" "$install_file"
    systemctl --user daemon-reload
    systemctl --user enable "$service_name"
}

function uninstall_service() {
    systemctl --user disable --now "$service_name" >/dev/null 2>&1 || true
    rm -f "$install_file"
    systemctl --user daemon-reload
}

case "$command" in
    install)
        install_service
        ;;
    uninstall)
        uninstall_service
        ;;
esac
