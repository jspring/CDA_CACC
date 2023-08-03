include capath_qnx.mk

PATH_OBJS += $(OBJDIR)/Leddar.o $(OBJDIR)/Modbus.o $(OBJDIR)/OS.o
#PATH_OBJS +=  $(OBJDIR)/Modbus.o $(OBJDIR)/OS.o

EXEC = $(OBJDIR)/leddar

all: $(OBJDIR) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)
clean:
	rm -f $(OBJDIR)/*.o *.err $(EXEC)

