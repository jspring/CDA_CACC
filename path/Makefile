# Makefile for California PATH shared code
#
# Copyright (c) 2008   Regents of the University of California
#
# CAPATH_MK_DEFS must be set in the environment to correct file of
# definitions forthe compile environment (lnx, bsd, qnx or sobos)
# Check in ../build for appropriate file.

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

# mtcp and Tilcon libraries are not built by default
PATH_LIBRARIES = $(LOCAL_DIR)/$(OBJDIR)/liblocals.a\
	$(DB_DIR)/$(OBJDIR)/libdb_clts.a

# not all libraries in sens and sqlite_eg trees may be built for all systems 
all: setup $(PATH_LIBRARIES) 
	make -C $(CAPATH_DIR)/clt
	make -C  $(CAPATH_DIR)/sens

$(LOCAL_DIR)/$(OBJDIR)/liblocals.a: $(wildcard local/*.[ch])
	make -C $(LOCAL_DIR)

$(DB_DIR)/$(OBJDIR)/libdb_clts.a: $(wildcard db/*.[ch])
	make -C $(DB_DIR)

clean:
	make -C $(LOCAL_DIR) clean
	make -C $(DB_DIR) clean
	make -C $(CAPATH_DIR)/clt clean
	make -C $(CAPATH_DIR)/sens clean

# This rule makes sure that all files are set up correctly for
# the system specifed by $(CAPATH_MK_DEFS), in case you are changing
# back and forth between native and cross-compiles in the same
# checkout tree. Libraries and executables will be put in subdirectories
# as specified by OBJDIR. This definition is also used to specify
# a system specific target.

setup: $(OBJDIR)
#	(cd $(LOCAL_DIR); ln -f -s $(SYS_OS_HEADER_FILE) sys_os.h)

# The following targets can be used to add any additional configuration
# required to set up the compile for different target architectures 

cyg:
	(echo 'Windows Cygwin compile' > setup.$(OBJDIR))

lnx:
	(cp $(CAPATH_DIR)/build/Makefile.$(OBJDIR) $(DISTRIB_DIR)/Makefile)
	(echo 'Native Linux compile' > setup.$(OBJDIR))

qnx:
#	(cp $(CAPATH_DIR)/build/Makefile.$(OBJDIR) $(DISTRIB_DIR)/Makefile)
	(echo 'Native QNX6 compile' > setup.$(OBJDIR))
