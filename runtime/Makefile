ROOT_DIR := $(realpath ..)
include $(ROOT_DIR)/config.mk

all: stub rtlsim maxhpc simx opae

stub:
	$(MAKE) -C stub

simx:
	$(MAKE) -C simx

rtlsim:
	$(MAKE) -C rtlsim

maxhpc:
	$(MAKE) -C maxhpc

opae:
	$(MAKE) -C opae

xrt:
	$(MAKE) -C xrt

clean:
	$(MAKE) clean -C stub
	$(MAKE) clean -C simx
	$(MAKE) clean -C rtlsim
	$(MAKE) clean -C maxhpc
	$(MAKE) clean -C opae
	$(MAKE) clean -C xrt

.PHONY: all stub simx rtlsim maxhpc opae xrt clean
