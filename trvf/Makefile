# compilation options
CXX = g++ -Wall -fPIC 
CXXFLAGS =  `pkg-config --cflags stage` 
LINKFLAGS = `pkg-config --libs stage` 

all: coordination.so createScenario draw.so

ConfigFile.o: ConfigFile.cpp ConfigFile.h
	$(CXX) -c ConfigFile.cpp -o ConfigFile.o

FinalLog.o: FinalLog.cpp FinalLog.h commonConfig.h util.h util.cpp
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c FinalLog.cpp -o FinalLog.o -lm

createScenario: createScenario.cpp ConfigFile.h ConfigFile.o commonConfig.h
	$(CXX) -g ConfigFile.o createScenario.cpp -o createScenario

draw.so: draw.cpp commonConfig.h ConfigFile.h util.h ConfigFile.o util.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o draw.so draw.cpp ConfigFile.o util.o

forcevisualizer.o: forcevisualizer.cpp forcevisualizer.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c forcevisualizer.cpp -o forcevisualizer.o

Robot.o: Robot.cpp Robot.h FinalLog.h commonConfig.h commonDefs.h  ConfigFile.h util.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c Robot.cpp -o Robot.o 

util.o: util.cpp util.h
	$(CXX) -c util.cpp -o util.o

coordination.so: util.o forcevisualizer.o Robot.o FinalLog.o ConfigFile.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -o coordination.so forcevisualizer.o FinalLog.o Robot.o ConfigFile.o util.o

clean:
	@rm -f *.o *.so  createScenario
	@find -regextype posix-awk  -regex .*configExperiment.*.ini\|.*automatic.*\|.*experimentsLog.* -delete
