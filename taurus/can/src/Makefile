
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

PATH_OBJS = $(OBJDIR)/taurus_lib.o
PATH_INCLUDES = taurus_can.h
PATH_CFLAGS += -I../../../steinhoff -I../../../vehcomm -I../../../include -fPIC

#EXEC = $(OBJDIR)/taurus_can  $(OBJDIR)/clt_vars_taurus $(OBJDIR)/taurus_can_substitute_389_and_393 $(OBJDIR)/send_brake_level 
EXEC = $(OBJDIR)/clt_vars_taurus $(OBJDIR)/taurus_can

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

install:
	cp -p $(EXEC) ../../test
clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
