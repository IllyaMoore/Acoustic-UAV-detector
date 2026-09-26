# Host build of the DSP component (plus the hardware-free report formatter):
# unit tests + the doa_cli replay tool.
#   make test      build and run the unit tests
#   make doa_cli   build the WAV replay tool
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror
DSP      := ../../components/dsp
INC      := -I$(DSP)/include -I../../main
SRCS     := $(wildcard $(DSP)/src/*.cpp)
BUILD    := build

all: $(BUILD)/test_dsp $(BUILD)/doa_cli

$(BUILD)/test_dsp: test_dsp.cpp ../../main/report.cpp $(SRCS) $(wildcard $(DSP)/include/dsp/*.hpp) check.hpp synth.hpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) test_dsp.cpp ../../main/report.cpp $(SRCS) -o $@

$(BUILD)/doa_cli: doa_cli.cpp $(SRCS) $(wildcard $(DSP)/include/dsp/*.hpp)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) doa_cli.cpp $(SRCS) -o $@

test: $(BUILD)/test_dsp
	./$(BUILD)/test_dsp

doa_cli: $(BUILD)/doa_cli

clean:
	rm -rf $(BUILD)

.PHONY: all test doa_cli clean
