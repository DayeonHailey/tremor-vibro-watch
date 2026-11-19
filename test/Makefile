# Host build (no device needed). On Windows use mingw32-make from WinLibs.
#   make          build
#   make check    run tests
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
CPPFLAGS += -I../include
CORE      = ../src/tremor_gate.cpp

all: test_gate replay

test_gate: test_gate.cpp $(CORE) ../include/tremor_gate.h ../include/haptic_controller.h
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) test_gate.cpp $(CORE) -o $@

replay: replay.cpp $(CORE) ../include/tremor_gate.h
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) replay.cpp $(CORE) -o $@

check: test_gate
	./test_gate

clean:
	rm -f test_gate replay test_gate.exe replay.exe

.PHONY: all check clean
