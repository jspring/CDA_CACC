#!/bin/sh

ssh admin@172.16.1.128 "while [ 1 ] ; do  cat /proc/sys/gps/nmea | grep TUDE; done"
