DSONAME = nvFlexDop_$(HOUDINI_MAJOR_RELEASE)$(HOUDINI_MINOR_RELEASE).so
SOURCES = $(addprefix $(PWD)/, $(shell echo nvFlexDop/*.cpp))
CC = $(CXX) -std=c++11

INSTDIR = $(PWD)/x64/linux64
INCDIRS = -I$(NVFLEX_DIR)/include
LIBDIRS = -L$(NVFLEX_DIR)/lib/linux64

include $(HFS)/toolkit/makefiles/Makefile.gnu

