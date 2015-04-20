# Makefile
# 
# Reflexxes GmbH
# Sandknoell 7
# D-24805 Hamdorf
# GERMANY
#
# http://www.reflexxes.com
#
# Author : Torsten Kroeger, <info@reflexxes.com>
# Date   : January 2012
#
#
#


include Makefile.global

export

# ********************************************************************************
# ********************************************************************************
# LINUX
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),__LINUX__)

include TargetDependencies.global

export


# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all:
	@echo $(LINE2)
	@echo Building the entire project tree... [$(VARIANT)]
	@echo Operating system: $(OS_NAME)
	cd ReflexxesTypeII;make $(VARIANT)
	cd RMLPositionSampleApplications;make $(VARIANT)
	cd RMLVelocitySampleApplications;make $(VARIANT)
	@echo This is the end of the build process [$(VARIANT)].
	@echo $(LINE2)


# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************
do_clean:
	@echo $(LINE2)
	@echo Cleaning up the entire project tree... [$(VARIANT)]
	@echo Operating system: $(OS_NAME)
	cd ReflexxesTypeII;make $(VARIANT)
	cd RMLPositionSampleApplications;make $(VARIANT)
	cd RMLVelocitySampleApplications;make $(VARIANT)	
	@echo This is the end of the clean up process [$(VARIANT)].
	@echo $(LINE2)

include ExternalTargets.global

endif



