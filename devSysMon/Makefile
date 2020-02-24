TOP=..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC = devSysMon

DBD += devSysMon.dbd

# Add locally compiled object code
devSysMon_SRCS += devSysMon.c
# The following adds support from base/src/vxWorks
OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

devSysMon_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

