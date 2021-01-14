#!/bin/bash

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

sudo cp $scriptdir/config/10-cabot.rules /etc/udev/rules.d/10-cabot.rules 
