# 호스트 빌드 (장치 불필요). Windows는 WinLibs의 mingw32-make 사용.
#   make          빌드
#   make check    테스트 실행
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
