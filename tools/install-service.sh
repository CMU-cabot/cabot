#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

if [ $(id -u) -eq 0 ]; then
   echo "please do not run as root: $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../
projectdir=`pwd`
project=$(basename $projectdir)

# Common variables
CABOT_DIR_LINK="/opt/cabot"
PROJECT_DIR_LINK="/opt/$project"
INSTALL_DIR="$HOME/.config/systemd/user"
SYS_INSTALL_DIR="/etc/systemd/system"
SERVICES_DIR="$scriptdir/config"
CABOT_SERVICE_NAME="cabot.service"
CABOT_SYSTEM_SERVICES_NAME="cabot-config.service check-bluetooth.service cabot-plugin.service"

# Print error messages in red
function err {
    >&2 red "[ERROR] $*"
}

# Print text in red
function red {
    echo -en "\033[31m"  ## red
    echo "$@"
    echo -en "\033[0m"  ## reset color
}

# Print text in blue
function blue {
    echo -en "\033[36m"  ## blue
    echo "$@"
    echo -en "\033[0m"  ## reset color
}

# Sleep function that doesn't block other processes (non-critical)
function snore() {
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

# Display help message
function help() {
    echo "Usage:"
    echo "  $0 <command>"
    echo ""
    echo "Commands:"
    echo "  -h          Show this help message."
    echo "  -c          Check services."
    echo "  -i          Install services."
    echo "  -u          Uninstall services."
}

command=

# Ensure only one command is chosen
function check_command() {
    if [[ -n $command ]]; then
        red "Cannot specify multiple commands."
        help
        exit 1
    fi
}

# Parse arguments
while getopts "chiu" arg; do
    case $arg in
        c)
            check_command
            command=check
            ;;
        h)
            help
            exit 0
            ;;
        i)
            check_command
            command=install
            ;;
        u)
            check_command
            command=uninstall
            ;;
        *)
            red "Invalid option: -$arg"
            help
            exit 1
            ;;
    esac
done

if [[ -z $command ]]; then
    red "No command selected. Please specify a command."
    help
    exit 1
fi


#----------------------------------------------------------
# Functions for each command
#----------------------------------------------------------

# Check services - add actual checks as needed
function check() {
    if [[ $PROJECT_DIR_LINK != $CABOT_DIR_LINK ]]; then
        blue "- Checking $CABOT_DIR_LINK"
        if [[ -e $CABOT_DIR_LINK ]]; then
            blue "  - $CABOT_DIR_LINK is created"
        else
            red "  - $CABOT_DIR_LINK is not created"
        fi
    fi
    blue "- Checking $PROJECT_DIR_LINK"
    if [[ -e $PROJECT_DIR_LINK ]]; then
        blue "  - $PROJECT_DIR_LINK is created"
    else
        red "  - $PROJECT_DIR_LINK is not created"
    fi

    blue "- Checking user service"
    if systemctl --user list-unit-files --type=service --all | grep "$CABOT_SERVICE_NAME"; then
        blue "  - $CABOT_SERVICE_NAME is installed"
        systemctl --user status $CABOT_SERVICE_NAME --no-pager
    else
        red "  - $CABOT_SERVICE_NAME is not installed"
    fi
    blue "- Checking system service"
    for service in $CABOT_SYSTEM_SERVICES_NAME; do
        if systemctl list-unit-files --type=service --all | grep "$service"; then
            blue "  - $service is installed"
            systemctl status $service --no-pager
        else
            red "  - $service is not installed"
        fi
    done
    return 0
}

# Install services
function install() {
    blue "- Installing services..."

    # Create a symlink if not already present
    blue "- Making $PROJECT_DIR_LINK symlink to $projectdir"
    if [[ ! -e "$PROJECT_DIR_LINK" ]]; then
        if ! sudo ln -sf "$projectdir" "$PROJECT_DIR_LINK"; then
            err "Failed to create symlink "$PROJECT_DIR_LINK" -> $projectdir"
            exit 1
        fi
    fi
    if [[ $CABOT_DIR_LINK != $PROJECT_DIR_LINK ]]; then
        blue "- Making $CABOT_DIR_LINK symlink to $PROJECT_DIR_LINK"
        if [[ ! -e "$CABOT_DIR_LINK" ]]; then
            if ! sudo ln -sf "$PROJECT_DIR_LINK" "$CABOT_DIR_LINK"; then
                err "Failed to create symlink "$CABOT_DIR_LINK" -> $PROJECT_DIR_LINK"
                exit 1
            fi
        fi
    fi

    blue "- Making $INSTALL_DIR for user service"
    mkdir -p "$INSTALL_DIR"

    # Check if ROS_DISTRO is set
    if [[ -z "$ROS_DISTRO" ]]; then
        # try to find a
        red "ROS_DISTRO is not set, so try to find the lastest ROS2 distro in the system"
        source $(find /opt/ros/ -maxdepth 2 -name setup.bash -exec grep -l ament {} + | sort -r | head -1)
    else
        source /opt/ros/$ROS_DISTRO/setup.bash
    fi
    if [[ -z "$ROS_DISTRO" ]]; then
        red "ROS_DISTRO is not set and could not be found"
        exit 1
    else
        blue "$ROS_DISTRO is found"
    fi

    # Replace paths in service file
    if [[ ! -f "$SERVICES_DIR/$CABOT_SERVICE_NAME" ]]; then
        err "Service template not found at $SERVICES_DIR/$CABOT_SERVICE_NAME"
        exit 1
    fi

    blue "- Installing $CABOT_SERVICE_NAME"
    #sed "s|%WORK_DIR%|$PROJECT_DIR_LINK|g" "$USER_SERVICES_DIR/$CABOT_SERVICE_NAME" \
    sed "s|%ROS_DISTRO%|$ROS_DISTRO|" "$SERVICES_DIR/$CABOT_SERVICE_NAME" > "$INSTALL_DIR/$CABOT_SERVICE_NAME"

    systemctl --user daemon-reload
    # systemctl --user enable $CABOT_SERVICE_NAME # do not enable here, started by cabot-app-server

    for service in $CABOT_SYSTEM_SERVICES_NAME; do
        # Install cabot-config.service for system wide
        if [[ ! -f "$SERVICES_DIR/$service" ]]; then
            err "Service template not found at $SERVICES_DIR/$service"
            exit 1
        fi

        blue "- Installing $service"
        MOUNTPOINT=`stat -c %m $projectdir`
        sed "s|# RequiresMountsFor =|RequiresMountsFor = $MOUNTPOINT|" "$SERVICES_DIR/$service" | sudo tee "$SYS_INSTALL_DIR/$service" > /dev/null
        sudo systemctl daemon-reload
        sudo systemctl enable $service --now
    done
    blue "- Installation complete."

    return 0
}


# Uninstall services
function uninstall() {
    blue "Uninstalling services..."

    # Remove systemd services if they exist
    if systemctl --user list-unit-files --type=service --all | grep -q "$CABOT_SERVICE_NAME"; then
        systemctl --user stop $CABOT_SERVICE_NAME || true
        systemctl --user disable $CABOT_SERVICE_NAME || true
        rm -f "$INSTALL_DIR/$CABOT_SERVICE_NAME"
        systemctl --user daemon-reload
    fi

    # Remove config service
    for service in $CABOT_SYSTEM_SERVICES_NAME; do
        if systemctl list-unit-files --type=service --all | grep -q "$service"; then
            sudo systemctl stop $service || true
            sudo systemctl disable $service || true
            sudo rm -f "$SYS_INSTALL_DIR/$service"
            sudo systemctl daemon-reload
        fi
    done

    # Remove symlink if exists
    if [[ $CABOT_DIR_LINK != $PROJECT_DIR_LINK ]]; then
        if [[ -L "$CABOT_DIR_LINK" ]]; then
            sudo rm -f "$CABOT_DIR_LINK"
        fi
    fi
    # Remove symlink if exists
    if [[ -L "$PROJECT_DIR_LINK" ]]; then
        sudo rm -f "$PROJECT_DIR_LINK"
    fi

    blue "Uninstallation complete."
    return 0
}

# Check prerequisites
function prerequisite_check() {
    # Check if systemctl is available
    blue "- Check if systemctl is available"
    if ! command -v systemctl &> /dev/null; then
        err "systemctl is not available. Please install systemd or run on a system with systemd."
        exit 1
    fi

    # If NVIDIA utilities are available, adjust sudoers for nvidia-smi
    if command -v nvidia-smi &>/dev/null; then
        blue "- Check if nvidia-smi's permission"
        USERNAME=$(id -un)
        SUDOER_FILE="/etc/sudoers.d/$USERNAME"
        if ! grep -q "/usr/bin/nvidia-smi" "$SUDOER_FILE" 2>/dev/null; then
            sudo tee "$SUDOER_FILE" > /dev/null <<- EOF
Cmnd_Alias USERCOMMANDS = /usr/bin/nvidia-smi
$USERNAME ALL=(ALL) NOPASSWD: USERCOMMANDS
EOF
        else
            blue "  - The sudoers file for $USERNAME already contains the required permissions."
        fi
    fi
}

blue "Running prerequisite_check"
prerequisite_check
blue "Running $command"
$command
exit $?
