
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

PATH_INCLUDES = pixell.h
PATH_CFLAGS += -I../../../../local -fPIC

#EXEC =  $(OBJDIR)/read_pixell $(OBJDIR)/clt_vars_pixell
EXEC =  $(OBJDIR)/read_pixell

all: $(OBJDIR) $(PATH_INCLUDES) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)
