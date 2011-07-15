ifndef TARGET
TARGET=sensinode
endif

# Make absolutely certain that you specify your device here
DEFINES=MODEL_N601,PROJECT_CONF_H

# We need uIPv6, therefore we also need banking
HAVE_BANKING=1
UIP_CONF_IPV6=1

# Do we need viztool functionality?
VIZTOOL_ENABLED=1

ifeq ($(VIZTOOL_ENABLED), 1)
APPS += viztool
DEFINES += ,VIZTOOL_ENABLED
endif

PROJECT_SOURCEFILES += slip-bridge.c

CONTIKI_PROJECT = border-router
all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
