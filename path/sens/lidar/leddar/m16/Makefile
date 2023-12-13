#include $(CAPATH_MK_DEFS)

include capath_qnx.mk

PATH_OBJS += $(OBJDIR)/Leddar.o $(OBJDIR)/Modbus.o $(OBJDIR)/OS.o
#PATH_LIBS += -lLeddarC
#PATH_LDFLAGS += -LLeddarSDK3.2.0_x86/Release

#EXEC = $(OBJDIR)/Rs485Demo $(OBJDIR)/leddar
EXEC = $(OBJDIR)/leddar

all: $(OBJDIR) $(PATH_OBJS) $(EXEC)

$(OBJDIR):
	mkdir -p $(OBJDIR)
clean:
	rm $(OBJDIR)/*.o $(OBJDIR)/Rs485Demo $(OBJDIR)/leddar
