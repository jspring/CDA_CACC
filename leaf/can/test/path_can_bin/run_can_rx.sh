#!/bin/sh
slay can_rx
slay accord_can
slay clt_vars_accord
slay db_slv

db_slv -Q -S `hostname` &
/home/qnxuser/path_can_bin/can_rx -p 2 -cv 
#/home/qnxuser/path_can_bin/clt_vars_accord &
#/home/qnxuser/path_can_bin/accord_can &
