include ../common.mk

MAXHPC_DIR := $(VORTEX_HOME)/ecpix-5/vx/sw

DESTDIR ?= $(CURDIR)

SRC_DIR := $(VORTEX_HOME)/runtime/maxhpc

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors
CXXFLAGS += -I$(INC_DIR) -I$(COMMON_DIR) -I$(ROOT_DIR)/hw -I$(COMMON_DIR) -I$(SIM_DIR)/common -I$(MAXHPC_DIR)
CXXFLAGS += -DXLEN_$(XLEN)
CXXFLAGS += -DMAXHPC

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

# Dump perf stats
CXXFLAGS += -DDUMP_PERF_STATS

LDFLAGS += -shared -pthread
LDFLAGS += -L$(DESTDIR)

SRCS := $(SRC_DIR)/vortex.cpp $(COMMON_DIR)/utils.cpp $(MAXHPC_DIR)/maxhpc.cpp

# Debugigng
ifdef DEBUG
	CXXFLAGS += -g -O0
else    
	CXXFLAGS += -O2 -DNDEBUG
endif

# Enable perf counters
ifdef PERF
	CXXFLAGS += -DPERF_ENABLE
endif

PROJECT := libvortex.so

all: $(DESTDIR)/$(PROJECT)
	
$(DESTDIR)/$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $@

clean:
	rm -rf $(DESTDIR)/$(PROJECT) *.o
