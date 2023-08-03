#!/bin/sh

cd /home/qnxuser/path_can_bin

echo admin/lapidar1X
ssh admin@172.16.1.128 cat /dev/gps_dev | grep GPRMC | ./gpssetdate -v
