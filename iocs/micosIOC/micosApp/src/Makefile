TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = micos
# micos.dbd will be created and installed
DBD += micos.dbd

# micos.dbd will be made up from these files:
micos_DBD += base.dbd

# Include dbd files from all support applications:
#micos_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#micos_LIBS += xxx

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

