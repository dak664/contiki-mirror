ifndef TARGET
TARGET=sensinode
endif

# Make absolutely certain that you specify your device here
DEFINES=MODEL_N601,PROJECT_CONF_H

# We need uIPv6, therefore we also need banking
HAVE_BANKING=1
UIP_CONF_IPV6=1

# Make sure we pre-process with UIP_CONF_ROUTER 1 and SLIP_ARCH_CONF_ENABLE 1
CFLAGS += -DUIP_CONF_ROUTER=1 -DSLIP_ARCH_CONF_ENABLE=1

# Use the large memory model because we are using printfs
MEMORY_MODEL=--model-large

PROJECT_SOURCEFILES += slip-bridge.c

CONTIKI_PROJECT = border-router
all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
