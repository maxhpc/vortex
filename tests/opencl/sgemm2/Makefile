ROOT_DIR := $(realpath ../../..)
include $(ROOT_DIR)/config.mk

PROJECT := sgemm2

SRC_DIR := $(VORTEX_HOME)/tests/opencl/$(PROJECT)

SRCS := $(SRC_DIR)/main.cc

CXXFLAGS += -I$(SRC_DIR)

common.h: $(SRC_DIR)/common.h 
	cp $< $@

setup: common.h
USE_SETUP := yes

OPTS ?= -n32

include ../common.mk
