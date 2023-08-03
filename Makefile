
#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

all:
	make -C path 
	make -C steinhoff
	make -C accord
	make -C prius
	make -C taurus
	make -C argonne_cacc
	make -C vehcomm
	make -C hmi
clean:
	make -C path clean
	make -C steinhoff clean
	make -C accord clean
	make -C prius clean
	make -C taurus clean
	make -C argonne_cacc clean
	make -C vehcomm clean
	make -C hmi clean

install:
	if [ ! -d qnx7 ] ; then mkdir -p qnx7; fi
	cp -p ./path/db/qnx/db_slv $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./accord/can/src/qnx/accord_can $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./accord/can/src/qnx/clt_vars_accord $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./taurus/can/src/qnx/taurus_can $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./taurus/can/src/qnx/clt_vars_taurus $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./prius/can/src/qnx/prius_can $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./prius/can/src/qnx/clt_vars_prius $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./argonne_cacc/qnx/argonne_cacc $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./steinhoff/qnx/can_rx $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./steinhoff/qnx/can_tx $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./vehcomm/qnx/veh_snd $(QNX_TARGET)/x86_64/usr/bin
	cp -p ./vehcomm/qnx/veh_rcv $(QNX_TARGET)/x86_64/usr/bin
	cp -p $(QNX_TARGET)/../../bsp/BSP*/images/x86_64-generic.build qnx7
	cp -p $(QNX_TARGET)/../../bsp/BSP*/images/x86_64-generic.bin qnx7
	cp -p ./hmi/hmi_server/qnx/hmi_server $(QNX_TARGET)/x86_64/usr/bin
