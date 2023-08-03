#!/bin/sh
STR=$1
ls -ltr *$STR* |tail -1
