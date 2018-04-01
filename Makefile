CC=g++
CFLAGS=-I .
DEPS = trajectory.hpp

%: %.cpp
	$(CC) -o $@ $(DEPS) $^ $(CFLAGS)
