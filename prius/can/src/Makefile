
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk
#include $(CAPATH_MK_DEFS)

PATH_OBJS += $(OBJDIR)/prius_lib.o
PATH_INCLUDES = prius_can.h
PATH_CFLAGS += -I../../../steinhoff -I../../../vehcomm -I../../../include -fPIC

EXEC = $(OBJDIR)/prius_can_set $(OBJDIR)/prius_can $(OBJDIR)/clt_vars_prius

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

install:
	cp -p $(EXEC) ../../test
clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
