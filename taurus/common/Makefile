# The "common" directory contains code utilities and header files
# shared by other code under long_ctl/src
 

include $(CAPATH_MK_DEFS)

KOMODO_DIR = /home/path/sens/can2usb/komodo/files/komodo-api-linux-i686-v1.00/c
PRIUS_DIR = /home/das3/veh/prius/can/src

PATH_LIBS += -ldl
PATH_CFLAGS += -fPIC
PATH_CFLAGS += -I$(PRIUS_DIR) -I/home/das3/src

# definitions of DISTRIB_DIR and CAN_DIR can be found in CAPATH_MK_DEFS

#TRUCKDIR = $(DISTRIB_DIR)/truckcontrol
COMMON_DIR = .
#JBUS_DIR = $(CAN_DIR)/jbus/
#COMM_DIR = $(TRUCKDIR)/long_ctl/src/vehcomm
#AVCS_DIR = $(TRUCKDIR)/long_ctl/src/avcs

CFLAGS = $(PATH_CFLAGS) -I$(COMMON_DIR)
#CFLAGS += -I$(AVCS_DIR)
#CFLAGS += -I$(COMM_DIR)

LINKFLAGS = $(PATH_LDFLAGS) -L$(COMMON_DIR) -L.

LINK = $(PATH_LINK)
#LONG_EXECS = test_utils long_out
LONG_EXECS = test_utils
LONG_LIB = liblong_utils.a
#LIBS = -llong_utils $(PATH_LIBS) -ljbus 
LIBS = -llong_utils $(PATH_LIBS)

all:    $(LONG_LIB) $(LONG_EXECS)

.c.o:
	$(CC) $(CFLAGS) -o $@ $<

$(LONG_LIB): long_utils.o
	$(PATH_AR) $(PATH_AR_FLAGS) $@ long_utils.o

test_utils:     $(LONG_LIB) test_utils.o
	$(LINK) $(LINKFLAGS) -o $@ test_utils.o $(LIBS)

#long_out:       $(LONG_LIB) long_out.o
#	$(LINK) $(LINKFLAGS) -o $@ long_out.o $(LIBS)

install: all
	if [[ ! -d ../test ]] ; then mkdir -p ../test; fi
	cp -f $(LONG_EXECS) $(TRUCKDIR)/test

clean:
	/bin/rm -f *.o  *.err $(LONG_EXECS) $(LONG_LIB)

distclean: clean
	cd $(TRUCKDIR)/test; /bin/rm -f $(LONG_EXECS)
