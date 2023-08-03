
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

PATH_CFLAGS += -g -O0 -finstrument-functions -I../../path/db -I../../path/local -I../../../common -I../../include -fPIC -I../../prius/can/src -I../../taurus/can/src -I../../accord/can/src -I../../steinhoff -I../../path/sens/gps/src -I../../vehcomm
PATH_LDFLAGS += -fPIC -L../lib
PATH_LIBS += -lprofilingS

EXEC = $(OBJDIR)/hmi_server

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir $(OBJDIR)

clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
