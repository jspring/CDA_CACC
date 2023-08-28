#!/bin/bash
#
# Do various things when the OS is first installed
#

# Create data directories
sudo mkdir /big
sudo mkdir /big/data
sudo chown jspring /big/data
sudo chgrp users /big/data
mkdir /big/data/olddata
mkdir /home/qnxuser/bin
mkdir /home/qnxuser/path_can_bin
