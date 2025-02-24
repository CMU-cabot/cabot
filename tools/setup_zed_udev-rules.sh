# This script will setup USB rules to open the ZED cameras without root access
# This can also be useful to access the cameras from a docker container without root (this script needs to be run on the host)
# NB: Running the ZED SDK installer will already setup those

# Print the commands
set -x
# Download the lightest installer
wget -q https://download.stereolabs.com/zedsdk/3.5/jp44/jetsons -O zed_installer.run
# Extracting only the file we're interested in
bash ./zed_installer.run --tar -x './99-slabs.rules'  > /dev/null 2>&1
sudo mv "./99-slabs.rules" "/etc/udev/rules.d/"
sudo chmod 777 "/etc/udev/rules.d/99-slabs.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger
