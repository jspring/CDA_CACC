#!/bin/sh

echo "Must execute as qnxuser"
cd /home/qnxuser/bin
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/scripts/* .

