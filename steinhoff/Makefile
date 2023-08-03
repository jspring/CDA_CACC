#\file Makefile for PATH CAN client library and example program
#
# Copyright (c) 2008   Regents of the University of California
#
# In order to use the default rules in the included $(CAPATH_MK_DEFS)
# file, symbolic links are created with the code files in the parent
# directory. 

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

HDRS = can_client.h $(CAN_DIR)/include/can_defs.h

LOCAL_LIB = $(OBJDIR)/libcapath_can.a

EXECS = $(OBJDIR)/can_tx $(OBJDIR)/can_rx #$(OBJDIR)/can_rx1_tx2
# $(OBJDIR)/can_tx_taurus_brake_only
CAN_OBJS = $(OBJDIR)/can_client_steinhoff.o

PATH_CFLAGS += -g -I$(CAN_DIR)/include -I$(CAN_DIR) -I../include

PATH_LIBS += -lcapath_can -lPKCAN 

PATH_LDFLAGS += -L$(CAN_DIR)/client/steinhoff/$(OBJDIR) -L./lib -L./$(OBJDIR)

all: $(OBJDIR) $(CAN_OBJS) $(LOCAL_LIB) $(EXECS) 

$(OBJDIR):
	#mkdir -p $(OBJDIR)
	#$(CAN_DIR)/client/mklinks $(CAN_DIR)


$(OBJDIR)/libcapath_can.a: $(CAN_OBJS)                                
	$(PATH_AR) $(PATH_AR_FLAGS)  $@ $(CAN_OBJS)    

install: all
	cp $(EXECS) $(CAN_DIR)/dachs_sja/test 

clean:
	rm -f $(LOCAL_LIB) $(EXECS) $(OBJDIR)/*.o $(OBJDIR)/*.err
