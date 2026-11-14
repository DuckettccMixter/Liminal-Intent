# Liminal Intent Pulsaret — VCV Rack 2 plugin
# Build from the MSYS2 MinGW 64-bit shell:  make RACK_DIR=<path to Rack-SDK>
RACK_DIR ?= ../..

# DSP headers (PulsaretPDCore.hpp, PulsaretHostVoice.hpp) are vendored in src/dsp so the
# repository builds on its own (the VCV Library build server only sees this folder).
FLAGS += -Isrc/dsp

# The core needs C++20 (std::numbers). Rack's compile.mk sets -std=c++11 first and appends
# EXTRA_CXXFLAGS afterwards, so the last -std wins.
EXTRA_CXXFLAGS += -std=c++20
# Rack's own helpers.hpp uses [=] lambdas that capture 'this', which C++20 flags as deprecated.
EXTRA_CXXFLAGS += -Wno-deprecated

SOURCES += $(wildcard src/*.cpp)

DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)

include $(RACK_DIR)/plugin.mk
