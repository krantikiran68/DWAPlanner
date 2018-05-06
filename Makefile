CC=g++
CFLAGS= -I . `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
DEPS = obstacle_distance.cpp

%: %.cpp
	$(CC) $(CFLAGS) -o $@ $(DEPS) $^ $(LIBS)
