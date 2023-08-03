# Makefile for California PATH sensor libraries and example prorams
#
# Copyright (c) 2008   Regents of the University of California
#

include capath_qnx.mk

SENS_LIBRARIES =\
	gps/src/$(OBJDIR)/libpath_gps.a \
	radar/$(OBJDIR)/libevt300.a 


all: includes lib/$(OBJDIR) $(SENS_LIBRARIES) $(SENS_DRIVERS) 
# Uncomment the following for QNX6, they break the Linux build.
#	make -C lidar/leddar

includes:
#	./make_includes

lib/$(OBJDIR):
	make -C lib

gps/src/$(OBJDIR)/libpath_gps.a: $(wildcard gps/src/*.[ch])
	make -C gps

lidar/ibeo_can/$(OBJDIR)/libibeo_utils.a: $(wildcard lidar/ibeo_can/*.[ch])
#	(cd lidar; make)

radar/$(OBJDIR)/libevt300.a: $(wildcard radar/*.[ch])
	make -C radar

clean:
	make -C gps clean
	make -C radar clean
	make -C lib clean
	make -C lidar/leddar clean
#	make -C lidar/denso clean
#	make -C lidar/mdl clean
