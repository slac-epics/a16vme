TOP = ..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_vxWorks += a16Vme
LIBRARY_RTEMS += a16Vme

# Library source files
a16Vme_SRCS += devA16Vme.c

# Link with the asyn and base libraries
a16Vme_LIBS += $(EPICS_BASE_IOC_LIBS)

# Install .dbd and .db files
DBD += devA16Vme.dbd
#DB_INSTALLS += xxa16Vme.vdb

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
