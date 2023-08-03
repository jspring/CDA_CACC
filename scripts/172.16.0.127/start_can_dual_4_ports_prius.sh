#!/bin/sh

slay can_dual
can_dual -k 10BB02874C811AC9E5BD7E59 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d1 -n CANDRV1 &
can_dual -k 10BB02874C811AC9E5BD7E59 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d2 -n CANDRV2 &
