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

include ../Makefile.global

export

# ********************************************************************************
# ********************************************************************************
# LINUX
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),__LINUX__)

include ../TargetDependencies.global

export

# ---------- Objects ----------

SRCS         := $(shell find $(SRC_DIR)/RMLVelocitySampleApplications -name '*.cpp')
OBJS         := $(patsubst $(SRC_DIR)/RMLVelocitySampleApplications/%.cpp, $(OBJ_DIR)/%.$(OBJECT_FILE_EXT), $(SRCS))
EXES         := $(patsubst $(SRC_DIR)/RMLVelocitySampleApplications/%.cpp, $(EXE_DIR)/%, $(SRCS))

$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/RMLVelocitySampleApplications/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@

# -------- OutputFiles --------

$(EXE_DIR)/%: $(OBJ_DIR)/%.$(OBJECT_FILE_EXT)
	@echo $(LINE1)
	$(CL) -o $@ $^ -lReflexxesTypeII

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(EXES)
	@echo Built directory contents
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	

# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************

do_clean:
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	rm -f $(OBJS)
	rm -f $(EXES)

include ../ExternalTargets.global

endif

	

