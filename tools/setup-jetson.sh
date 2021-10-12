#!/bin/sh


sudo apt update
sudo apt remove thunderbird libreoffice-* -y

sudo cp config/daemon.json /etc/docker/
sudo pkill -SIGHUP dockerd
sudo gpasswd -a $USER docker

# install docker-compose
sudo curl -L https://github.com/docker/compose/releases/download/v2.0.1/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

sudo apt update
sudo apt install --no-install-recommends -y \
	nvidia-docker2 \
	nvidia-cudnn8 libcublas-dev libcurand-dev-10-2 \
	python3-pip python3-setuptools
sudo pip3 install -U vcstool

sudo rm -rf/var/lib/apt/lists/*
sudo apt autoremove -y
sudo apt clean

