CC=g++
CFLAGS= -I . `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
DEPS = trajectory.cpp windowparam.cpp obstacle_distance.cpp

%: %.cpp
	$(CC) $(CFLAGS) -o $@ $(DEPS) $^ $(LIBS)
