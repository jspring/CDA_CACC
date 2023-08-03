#!/bin/sh
IPADDR=`ifconfig en0 | grep "inet " |gawk '{print $2}'` 
DSRCADDR=`echo $IPADDR | sed '{s/172.16.0/172.16.1/g}'` 


cd /home/qnxuser/path_can_bin
./veh_snd -A $IPADDR -a $DSRCADDR -u 1516 -vd
