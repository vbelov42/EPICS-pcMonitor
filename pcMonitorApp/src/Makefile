TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

#PROD_HOST = pcMonitor
#PROD_IOC is used to complie things for all architectures
PROD_IOC = pcMonitor

# pcMonitor.dbd will be created and installed
DBD += pcMonitor.dbd

# pcMonitor.dbd will be made up from these files:
pcMonitor_DBD += base.dbd
pcMonitor_DBD += devSysMon.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
pcMonitor_SRCS += pcMonitor_registerRecordDeviceDriver.cpp
pcMonitor_SRCS_DEFAULT += pcMonitorMain.cpp
pcMonitor_SRCS_vxWorks += -nil-

pcMonitor_LIBS += $(EPICS_BASE_IOC_LIBS)
pcMonitor_LIBS += devSysMon

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

