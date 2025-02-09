include ../common.mk

TARGET ?= opaesim

DESTDIR ?= $(CURDIR)

SYN_DIR := $(HW_DIR)/syn/altera/opae

SRC_DIR := $(VORTEX_HOME)/runtime/opae

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors
CXXFLAGS += -I$(INC_DIR) -I$(COMMON_DIR) -I$(ROOT_DIR)/hw -I$(DESTDIR)
CXXFLAGS += -DXLEN_$(XLEN)

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

# Dump perf stats
CXXFLAGS += -DDUMP_PERF_STATS

LDFLAGS += -shared -luuid -ldl -pthread

SRCS = $(SRC_DIR)/vortex.cpp $(SRC_DIR)/driver.cpp $(COMMON_DIR)/utils.cpp

# set up target types
ifeq ($(TARGET), opaesim)
	OPAESIM = $(DESTDIR)/libopae-c-sim.so
	CXXFLAGS += -DOPAESIM -I$(SIM_DIR)/opaesim
	LDFLAGS += -L$(DESTDIR) -lopae-c-sim
else
	CXXFLAGS += -I$(SYN_DIR)
	ifeq ($(TARGET), asesim)
		CXXFLAGS += -DASESIM
	else
		CXXFLAGS += -DFPGA
	endif
endif

# Debugigng
ifdef DEBUG
	CXXFLAGS += -g -O0
else    
	CXXFLAGS += -O2 -DNDEBUG
endif

# Enable scope logic analyzer
ifdef SCOPE
	CXXFLAGS += -DSCOPE	
	SRCS += $(COMMON_DIR)/scope.cpp
endif

# Enable perf counters
ifdef PERF
	CXXFLAGS += -DPERF_ENABLE
endif

PROJECT := libvortex.so

all: $(DESTDIR)/$(PROJECT)

$(DESTDIR)/libopae-c-sim.so:
	DESTDIR=$(DESTDIR) $(MAKE) -C $(ROOT_DIR)/sim/opaesim $(DESTDIR)/libopae-c-sim.so

$(DESTDIR)/$(PROJECT): $(SRCS) $(OPAESIM)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $@

clean:
	DESTDIR=$(DESTDIR) $(MAKE) -C $(ROOT_DIR)/sim/opaesim clean
	rm -rf $(DESTDIR)/$(PROJECT)
