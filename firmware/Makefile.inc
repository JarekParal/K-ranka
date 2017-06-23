APPL_COBJS += 

APPL_CXXOBJS += json11.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/lib-ev3cxx/Makefile
CPPFLAGS := $(CPPFLAGS) -Iatoms/include

endif
