# Makefile
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

TRUCK_DIR = $(DISTRIB_DIR)/truckcontrol
ASNDIR = ./BSM_dir
SERVERDIR = ./HIP_Server_Open_Source

PATH_CFLAGS += -I$(SERVERDIR)/Include -I$(SERVERDIR)/HIP_Server
PATH_CFLAGS += -I$(ASNDIR)
PATH_LDFLAGS += -L$(ASNDIR)
PATH_OBJS += $(ASNDIR)/*.o $(OBJDIR)/veh_lib.o
PATH_LIBS = -lasncodec $(PATH_CORE_LIBS)

EXEC =  $(OBJDIR)/veh_rcv $(OBJDIR)/veh_snd  $(OBJDIR)/veh_snd_taurus_itri $(OBJDIR)/veh_snd_test

all:    $(OBJDIR) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(ASNDIR)/*.o:
	make -C $(ASNDIR)
clean:
	rm -f $(ASNDIR)/*.o $(OBJDIR)/*.o $(OBJDIR)/*.err *.*~ $(EXEC)

install:
	cp ./BSM_dir/J2735_PATH_BSM.ASN .

# DO NOT DELETE

