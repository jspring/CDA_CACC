# Makefile - PATH GPS library and sample programs
#
# Copyright (c) 2008   Regents of the University of California
#

#include $(CAPATH_MK_DEFS)
include capath_qnx.mk

all: 
	make -C src
	make -C src install
	make -C examples

clean:
	make -C src clean
	make -C examples clean
