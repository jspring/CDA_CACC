#!/bin/sh

cd /home/qnxuser/path_can_bin

ssh rsu@172.16.1.120 gpspipe -r | ./gpssetdate -v 
