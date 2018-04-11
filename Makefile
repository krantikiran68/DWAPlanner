CC=g++
CFLAGS=-I .
DEPS = trajectory.cpp windowparam.cpp

%: %.cpp
	$(CC) -o $@ $(DEPS) $^ $(CFLAGS)
