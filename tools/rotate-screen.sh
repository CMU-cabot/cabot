#!/bin/bash

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

rotate="${1:-normal}"

xrandr --output "$(xrandr | grep " connected" | cut -d " " -f 1 | head -n 1)" --rotate "$rotate"
