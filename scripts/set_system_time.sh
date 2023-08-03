#!/bin/sh
ssh rsu@172.16.1.120 gpspipe -r | /home/qnxuser/bin/gpssetdate -D 
sleep 1
echo yes >0
sleep 1
echo rsuadmin >0
rtc -s hw
