
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

HDRS= $(SENS_INC_DIR)/path_gps_lib.h

PATH_CFLAGS += -I$(ATSC_INC_DIR) -I$(CICAS_SRC_DIR) -I../src/qnx
#PATH_CFLAGS += -g
PATH_LIBS = -lpath_gps $(PATH_CORE_LIBS) 
PATH_LDFLAGS += -L../src/qnx

EXEC = $(OBJDIR)/test_convert \
	$(OBJDIR)/tsprcv $(OBJDIR)/gpslatlong $(OBJDIR)/gpsdistance \
	 $(OBJDIR)/bcnrcv $(OBJDIR)/gpssetdate $(OBJDIR)/gpssnd \
	 $(OBJDIR)/gpsrcv $(OBJDIR)/tcp_client $(OBJDIR)/tcp_echo \
	 $(OBJDIR)/gpsdb $(OBJDIR)/gpsget $(OBJDIR)/macget \
	 $(OBJDIR)/gps2xy $(OBJDIR)/gpsdemo $(OBJDIR)/gpspoint \
	 $(OBJDIR)/rdgpsdb $(OBJDIR)/garmenable

all: $(OBJDIR) $(EXEC)

$(OBJDIR):
	mkdir $(OBJDIR)


clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)

