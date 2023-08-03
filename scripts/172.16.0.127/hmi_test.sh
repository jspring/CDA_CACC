
#!/bin/sh

slay -f db_slv
slay -f hmi_server
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1
./hmi_server

