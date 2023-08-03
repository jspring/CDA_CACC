
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

PATH_OBJS = $(OBJDIR)/accord_lib.o
PATH_INCLUDES = accord_can.h
PATH_CFLAGS += -I../../../steinhoff -I../../../vehcomm -I../../../include -fPIC

EXEC = $(OBJDIR)/accord_can $(OBJDIR)/clt_vars_accord

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

install:
	cp -p $(EXEC) ../../test
clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
