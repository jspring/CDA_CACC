#!/bin/sh
DATETIME=`ssh jspring@172.16.0.150 date +%m%d%H%M%Y.%S`
echo DATETIME $DATETIME
date $DATETIME
rtc -s hw
