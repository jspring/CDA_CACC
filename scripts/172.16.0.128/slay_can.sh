#!/bin/sh
slay -f -s KILL argonne_cacc
../bin/slayscript.sh leddar
slay -f -s KILL taurus_can
slay -f -s KILL taurus_can_substitute_389_and_393
slay -f -s KILL accord_can
slay -f -s KILL veh_snd
slay -f -s KILL veh_rcv
slay -f -s KILL can_tx
slay -f -s KILL can_rx
slay -f -s KILL clt_vars_accord
slay -f -s KILL clt_vars_taurus
slay -f -s KILL clt_vars_prius
slay -f -s KILL rdj1939
slay -f -s KILL jbussend
slay -f -s KILL gpsdb
slay -f -s KILL trk_wr
slay -f -s KILL db_slv
