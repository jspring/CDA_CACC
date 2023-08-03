#!/bin/sh
slay -f -s KILL argonne_cacc
slay -f -s KILL hmi_server
slay -f -s KILL accord_can
slay -f -s KILL taurus_can
slay -f -s KILL prius_can
slay -f -s KILL veh_snd
slay -f -s KILL veh_rcv
slay -f -s KILL gpsrcv
slay -f -s KILL can_tx
slay -f -s KILL can_rx
slay -f -s KILL clt_vars_accord
slay -f -s KILL clt_vars_taurus
slay -f -s KILL clt_vars_prius
slay -f -s KILL db_slv
