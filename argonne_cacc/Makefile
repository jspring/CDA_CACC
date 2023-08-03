
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

PATH_OBJS += $(OBJDIR)/long_control.o
PATH_CFLAGS += -g -O0 -finstrument-functions -I../../common -I../include -fPIC -I../prius/can/src -I../taurus/can/src -I../accord/can/src -I../steinhoff -I../vehcomm -I../hmi/hmi_server
PATH_LDFLAGS += -fPIC -L../lib
PATH_LIBS += -lprofilingS

EXEC = $(OBJDIR)/argonne_cacc

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir $(OBJDIR)

clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
