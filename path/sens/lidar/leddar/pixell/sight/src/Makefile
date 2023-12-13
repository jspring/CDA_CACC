
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

PATH_INCLUDES = leddar_SIGHT.h
PATH_CFLAGS += -I../../../../../../can/client/steinhoff -I../../../include -fPIC
PATH_OBJS +=  $(OBJDIR)/crc8_sae_j1850.o

EXEC =$(OBJDIR)/leddar_SIGHT $(OBJDIR)/clt_vars_SIGHT $(OBJDIR)/vehicle_speed

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
