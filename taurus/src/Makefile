
# Makefile - PATH GPS sample programs
#
# Copyright (c) 2012   Regents of the University of California
#

include $(CAPATH_MK_DEFS)

TAURUS_DIR = /home/das3/veh/qnx_vehicle_projects/taurus/can/src

PATH_OBJS = $(OBJDIR)/taurus_tables.o \

LIB = libtaurus_tables.so

PATH_LIBS += -ldl
PATH_CFLAGS += -fPIC
PATH_CFLAGS += -I$(TAURUS_DIR) -I/home/das3/src

EXEC = $(OBJDIR)/taurus_create

all: $(OBJDIR) $(PATH_OBJS) $(EXEC) $(LIB).1.0 

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(LIB).1.0:
	gcc -shared -Wl,-soname,libtaurus_tables.so.1 -o $(OBJDIR)/$(LIB).1.0 $(OBJDIR)/*.o
	ln -sf $(LIB).1.0 $(OBJDIR)/$(LIB)
	ln -sf $(LIB).1.0 $(OBJDIR)/$(LIB).1

install:
	cp -p $(EXEC) ../../test
clean:
	rm -f $(PATH_OBJS) $(EXEC) $(OBJDIR)/$(LIB)*
