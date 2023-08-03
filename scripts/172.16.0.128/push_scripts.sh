#!/bin/sh


echo "Must execute as qnxuser"
cd /home/qnxuser/bin
IPADDR=`ifconfig| grep inet | grep broadcast |gawk '{print $2}'`
ssh jspring@172.16.0.150 "mkdir -p /windows/Linux/Argonne_CACC/scripts/$IPADDR"
scp -p * jspring@172.16.0.150:/windows/Linux/Argonne_CACC/scripts/$IPADDR
