ifndef TARGET
TARGET=sensinode
endif

# Make absolutely certain that you specify your device here
DEFINES=MODEL_N601

# We need uIPv6, therefore we also need banking
HAVE_BANKING=1
UIP_CONF_IPV6=1

# Make sure we pre-process with UIP_CONF_ROUTER 1 and SLIP_ARCH_CONF_ENABLE 1
CFLAGS += -DUIP_CONF_ROUTER=1 -DSLIP_ARCH_CONF_ENABLE=1

# Use the huge memory model although large will work just as well
MEMORY_MODEL=--model-huge

PROJECT_SOURCEFILES += slip-bridge.c

CONTIKI_PROJECT = border-router.c
all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
