#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    xdotool key 'Super+Ctrl+f'
    snore 2
    xdotool key 'Super+Ctrl+q'
    snore 2
    kill -2 "$pid"
    exit
}
function snore() {
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read -r ${1:+-t "$1"} -u "$_snore_fd" || :
}

function help() {
    echo "Usage:"
    echo "-d <dir>         save directory"
    echo "-p <prefix>      prefix"
    echo "-s <second>      sleep for a second before start recording (default 10)"
}

save_dir=./
prefix=cabot_screen_recording
delay=10
pid=

scriptdir=$(dirname "$0")
cd "$scriptdir" || exit
scriptdir=$(pwd)

while getopts "hd:p:s:" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
        save_dir=$OPTARG
        ;;
    p)
        prefix=$OPTARG
        ;;
    s)
        delay=$OPTARG
        ;;
    *) ;;
    esac
done
shift $((OPTIND - 1))

save_dir=$(realpath "$save_dir")
echo "saving file to $save_dir/$prefix"

KAZAM_CONF_FILE=~/.config/kazam/kazam.conf
mv $KAZAM_CONF_FILE ${KAZAM_CONF_FILE}.back
cp "$scriptdir/tools/config/kazam.conf" $KAZAM_CONF_FILE
sed -i "s'autosave_video = .*'autosave_video = True'" $KAZAM_CONF_FILE
sed -i "s'autosave_video_dir = .*'autosave_video_dir = ${save_dir}'" $KAZAM_CONF_FILE
sed -i "s'autosave_video_file = .*'autosave_video_file = ${prefix}'" $KAZAM_CONF_FILE
cat $KAZAM_CONF_FILE

pushd "$save_dir" || exit
kazam &
pid=$!
echo "wait $delay"
snore "$delay"
xdotool key 'Super+Ctrl+r'
popd || exit

while true; do
    snore 1
done
