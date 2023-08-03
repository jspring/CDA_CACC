#!/bin/bash

FILE=$1
cat $FILE | sed '{s/:/ /g}' >$FILE.mod 
cat $FILE.mod | cut -d ' ' -f -3  >$FILE_time_only.txt
