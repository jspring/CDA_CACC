# common definitions for PATH builds on QNX6 systems 
#
# Copyright (c) 2008   Regents of the University of California
#

DISTRIB_DIR = ../../../../../..
SYS_OS_HEADER_FILE = $(DISTRIB_DIR)/path/local/sys_qnx6.h
OBJDIR = qnx
RELEASE_DIR = $(DISTRIB_DIR)/release

CAPATH_DIR = $(DISTRIB_DIR)/path
LOCAL_DIR = $(CAPATH_DIR)/local
DB_DIR = $(CAPATH_DIR)/db
CAN_DIR = $(DISTRIB_DIR)/can

# parts of sens library not yet ported to QNX
# but harmless to include definition as extra search area 
SENS_LIB_DIR = $(CAPATH_DIR)/sens/lib
SENS_INC_DIR = $(CAPATH_DIR)/sens/include

PATH_CC = ${QNX_HOST}/usr/bin/qcc -Vgcc_ntox86_64
PATH_CFLAGS += -c -D_GNU_SOURCE 
PATH_CFLAGS += -I. -I$(LOCAL_DIR) -I$(DB_DIR) -I$(SENS_INC_DIR) -I${QNX_TARGET}/usr/include
PATH_CFLAGS += -Wall
PATH_CFLAGS += -g

PATH_AR = $(PATH_CC)
PATH_LINK = $(PATH_CC)
PATH_LDFLAGS += -L$(LOCAL_DIR)/$(OBJDIR)  -L$(DB_DIR)/$(OBJDIR) \
		-L$(SENS_LIB_DIR)/$(OBJDIR)
PATH_LDFLAGS += -L/lib -L$(QNX_TARGET)/x86_64/lib/
#PATH_LDFLAGS += -L/opt/qnx7.0/qnx700/host/linux/x86_64/usr/lib/gcc/x86_64-pc-nto-qnx7.0.0/5.4.0
PATH_LDFLAGS += -L${QNX_HOST}/usr/lib/gcc/x86_64-pc-nto-qnx7.0.0/5.4.0

# core libs defined so that if required a new library
# can be added to the beginning to redefine PATH_LIBS
#PATH_CORE_LIBS = -ldb_clts -llocals -lsocket -lm -lc -lgcc
PATH_CORE_LIBS = -ldb_clts -llocals -lsocket -lm -lgcc
PATH_LIBS = $(PATH_CORE_LIBS)
PATH_AR_FLAGS = -A
# PATH_OBJS can be set up in individual Makefiles a desired
PATH_OBJS =


$(OBJDIR)/%.o: %.c $(HDRS)	
	$(PATH_CC) $(CFLAGS) $(PATH_CFLAGS) $*.c -o $@ 

.o:
	$(PATH_LINK) $(LDFLAGS) $(PATH_LDFLAGS) -o $@ $@.o $(PATH_OBJS) $(PATH_LIBS) 
#	$(PATH_LINK) $(LDFLAGS) $(PATH_LDFLAGS) -o $@ $@.o $(PATH_LIBS) 
	
