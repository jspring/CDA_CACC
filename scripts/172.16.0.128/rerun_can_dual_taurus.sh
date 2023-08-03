#!/bin/sh

slay can_dual
can_dual -k 090478E3A21A838E37EA7870 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d 1 -n CANDRV1 &
can_dual -k 090478E3A21A838E37EA7870 -B 0x00 -b 0x1c -C 0x00 -c 0x1c -d 2 -n CANDRV2 &
