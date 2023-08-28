#!/bin/sh

killall python2
killall prius_can
killall komodo_read_can_write_db
killall komodo_read_db_write_can
sleep .5
killall -s KILL komodo_read_can_write_db
killall -s KILL komodo_read_db_write_can
killall clt_vars
killall db_slv
/home/path/db/db_clean.sh
