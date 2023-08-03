#!/bin/sh

SCRIPT=$1
PID=`ps -elf | grep -v grep | grep $SCRIPT | gawk '{print $4}'`
echo pid $PID
slay -s KILL $PID
