TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for the source files that define template functions.
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

DBD += NDPluginCalib.dbd

INC += NDPluginCalib.h

LIBRARY_IOC += NDPluginCalib

NDPluginCalib_SRCS += NDPluginCalib.cpp

ifdef OPENCV_INCLUDE
  USR_INCLUDES += -I$(OPENCV_INCLUDE)
endif 
ifdef OPENCV_LIB
  NDPluginCalib_DIR += $(OPENCV_LIB)
endif

include $(AREA_DETECTOR)/ADCore/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

