CC=g++
CFLAGS=-I .
DEPS = trajectory.cpp

%: %.cpp
	$(CC) -o $@ $(DEPS) $^ $(CFLAGS)
