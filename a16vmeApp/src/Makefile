TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

a16Vme = $(TOP)
a16Vme_LIB = $(TOP)/lib/$(T_A)
a16Vme_BIN = $(TOP)/bin/$(T_A)

#=============================
# build an ioc application

PROD_IOC_vxWorks = a16VmeTest
PROD_IOC_RTEMS = a16VmeTest

DBD += a16VmeTest.dbd
# a16VmeTest.dbd will be made up from these files:
a16VmeTest_DBD += base.dbd
a16VmeTest_DBD += devA16Vme.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
a16VmeTest_SRCS += a16VmeTest_registerRecordDeviceDriver.cpp
a16VmeTest_SRCS_DEFAULT += a16VmeTestMain.cpp
a16VmeTest_SRCS_vxWorks += -nil-

#add a definition for each support application used by this application
a16VmeTest_LIBS += a16Vme
a16VmeTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

