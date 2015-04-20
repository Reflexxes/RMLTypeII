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

TypeIIRML_SRCS         := $(shell find $(SRC_DIR)/TypeIIRML -name '*.cpp')

TypeIIRML_OBJS         := $(patsubst $(SRC_DIR)/TypeIIRML/%.cpp, $(OBJ_DIR)/%.$(OBJECT_FILE_EXT), $(TypeIIRML_SRCS))


$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/TypeIIRML/%.cpp
	@echo $(LINE1)
	$(CC) $< -o $@

# -------- OutputFiles --------

$(LIB_DIR)/libReflexxesTypeII.$(LIB_FILE_EXT): $(TypeIIRML_OBJS)
	@echo $(LINE1)
	$(AR) $@ $^

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(LIB_DIR)/libReflexxesTypeII.$(LIB_FILE_EXT)
	@echo Built directory contents
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	

# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************

do_clean:
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	rm -f $(TypeIIRML_OBJS)
	rm -f $(LIB_DIR)/libReflexxesTypeII.$(LIB_FILE_EXT)

include ../ExternalTargets.global

endif

	