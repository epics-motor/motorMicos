TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = micos
# micos.dbd will be created and installed
DBD += micos.dbd

# micos.dbd will be made up from these files:
micos_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
micos_DBD += asyn.dbd
micos_DBD += drvAsynSerialPort.dbd
micos_DBD += drvAsynIPPort.dbd
#endif
micos_DBD += motorSupport.dbd
micos_DBD += devMicos.dbd

# Add all the support libraries needed by this IOC
micos_LIBS += Micos
micos_LIBS += motor
#ifdef ASYN
micos_LIBS += asyn
#endif
#ifdef SNCSEQ
micos_LIBS += seq pv
#endif

# micos_registerRecordDeviceDriver.cpp derives from micos.dbd
micos_SRCS += micos_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
micos_SRCS_DEFAULT += micosMain.cpp
micos_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#micos_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
micos_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

