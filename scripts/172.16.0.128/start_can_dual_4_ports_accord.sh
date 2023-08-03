#!/bin/sh

slay can_dual
can_dual -k 8D8E2A0F401136E7B1E0C6A6 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d1 -n CANDRV1 &
can_dual -k 8D8E2A0F401136E7B1E0C6A6 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d2 -n CANDRV2 &
