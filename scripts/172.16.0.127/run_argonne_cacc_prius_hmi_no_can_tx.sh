
#!/bin/sh
slay -f argonne_cacc
slay -f prius_can
slay -f can_tx
slay -f can_rx
slay -f clt_vars_prius
slay -f hmi_server
slay -f db_slv

cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` -v |grep 20017 &
sleep 1
./hmi_server -e 1 &
sleep 1
./clt_vars_prius &
sleep 1
./can_rx -p 1 -cv >can1_rx_out.txt &
sleep 1
#./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -v -n 4000 >can_tx_out_brake.txt &
sleep 1
./prius_can -v >prius_can_out.txt &
sleep 1
./argonne_cacc >/home/qnxuser/data/argonne_cacc.txt &
tail -f /home/qnxuser/data/argonne_cacc.txt
