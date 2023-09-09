# Makefile
#
# Copyright (c) 2008   Regents of the University of California
#

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

TRUCK_DIR = $(DISTRIB_DIR)/truckcontrol
GPS_DIR = $(DISTRIB_DIR)/path/sens/gps/src
ASNDIR = ./BSM_dir
SERVERDIR = ./HIP_Server_Open_Source
XYL_DIR = ../xyl


PATH_CFLAGS += -I$(SERVERDIR)/Include -I$(SERVERDIR)/HIP_Server
PATH_CFLAGS += -I$(ASNDIR) -I$(GPS_DIR) -I$(XYL_DIR)
PATH_LDFLAGS += -L$(ASNDIR)
PATH_OBJS += $(ASNDIR)/*.o $(OBJDIR)/veh_lib.o
PATH_LIBS = -lasncodec $(PATH_CORE_LIBS)


#EXEC =  $(OBJDIR)/veh_rcv $(OBJDIR)/veh_snd $(OBJDIR)/veh_snd_test $(OBJDIR)/dvi_rcv $(OBJDIR)/dvi_snd $(OBJDIR)/dvi_snd_db_test $(OBJDIR)/settime2trucksignal $(OBJDIR)/msleep
EXEC =  $(OBJDIR)/veh_snd $(OBJDIR)/veh_rcv $(OBJDIR)/settime2trucksignal $(OBJDIR)/msleep

all:    $(OBJDIR) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir $(OBJDIR)
#	mkdir -p $(OBJDIR)

$(ASNDIR)/*.o:
	make -C $(ASNDIR)
#	rm $(ASNDIR)/converter-example.o
clean:
	rm -f $(ASNDIR)/*.o $(OBJDIR)/*.o $(OBJDIR)/*.err *.*~ $(EXEC)

install:
	cp $(EXEC) $(TRUCK_DIR)/test
	cp ./BSM_dir/J2735_PATH_BSM.ASN .

# DO NOT DELETE

