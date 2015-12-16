CONTIKI_PROJECT = range
all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += signal.c usartinterface.c ultrasonic_serial.c cc2520-arch-sfd.c buffer.c
PROJECT_SOURCEFILES += packet.c crc.c comm.c

# Quadcopter Range Board
#TARGET = stm32-bv-copter

# Anchors
TARGET = stm32-bv

PRINTF_VCP = 0

# Anchor slot 0
#NODEID = 2
# Anchor slot 1
#NODEID = 3
# Anchor slot 3
#NODEID = 4
# Anchor slot 4
NODEID = 5

UIP_CONF_IPV6 = 1
APPS = webserver
LDFLAGS += -lm
USE_DSP_LIB = 1

DEBUG_BUILD = 0

CONTIKI = ../Lib/contiki-bv
include $(CONTIKI)/Makefile.include
