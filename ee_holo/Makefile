# path to common directory for all simulations
COMMON_DIR = ../common

# compilation options
CXX = g++
CXXFLAGS = -g -Wall `pkg-config --cflags stage` -fPIC
LINKFLAGS = `pkg-config --libs stage` 

# file paths
final_logs_objects = $(COMMON_DIR)/FinalLog.h $(COMMON_DIR)/FinalLog.cpp
connection_objects = $(COMMON_DIR)/connectionlocal.h $(COMMON_DIR)/connectionlocal.cpp
force_visualiser_objects = $(COMMON_DIR)/forcevisualizer.cpp $(COMMON_DIR)/forcevisualizer.h
commons = $(COMMON_DIR)/commonConfig.h $(COMMON_DIR)/commonMethods.cpp \
$(COMMON_DIR)/commonDefs.h $(COMMON_DIR)/commonMembers.cpp \
$(COMMON_DIR)/regionMembers.cpp $(COMMON_DIR)/regionMethods.cpp $(COMMON_DIR)/forcevisualizer.h \
$(COMMON_DIR)/util.h $(COMMON_DIR)/util.cpp

run: all

all: coordination.so createScenario draw.so

draw.so: $(COMMON_DIR)/draw.cpp draw.cpp wiseRobot.h $(COMMON_DIR)/commonConfig.h $(COMMON_DIR)/util.h $(COMMON_DIR)/util.cpp util.o
	$(CXX) $(CXXFLAGS) -shared -Wl,-soname,draw.so -o draw.so draw.cpp  $(LINKFLAGS) util.o

createScenario: createScenario.cpp
	$(CXX) createScenario.cpp -o createScenario

coordination.so: forcevisualizer.o connectionlocal.o wiseRobot.o FinalLog.o util.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -shared -Wl,-soname,coordination.so -o coordination.so forcevisualizer.o util.o FinalLog.o connectionlocal.o wiseRobot.o 

forcevisualizer.o: $(force_visualiser_objects)
	$(CXX) $(COMMON_DIR)/forcevisualizer.cpp -c -o forcevisualizer.o  $(CXXFLAGS) $(LINKFLAGS)

wiseRobot.o: wiseRobot.cpp wiseRobot.h $(commons) $(final_logs_objects) $(connection_objects)
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -c wiseRobot.cpp

util.o: $(COMMON_DIR)/util.h $(COMMON_DIR)/util.cpp
	$(CXX) -c $(COMMON_DIR)/util.cpp -o util.o $(CXXFLAGS) $(LINKFLAGS)

connectionlocal.o: $(connection_objects)
	$(CXX) -c  $(COMMON_DIR)/connectionlocal.cpp  $(CXXFLAGS) $(LINKFLAGS)

FinalLog.o: $(final_logs_objects)
	$(CXX) -c  $(COMMON_DIR)/FinalLog.cpp $(CXXFLAGS) $(LINKFLAGS)

clean:
	@rm -f *.o *.so  server createScenario
	@find -regextype posix-awk  -regex .*configExperiment.*.ini\|.*automatic.*\|.*experimentsLog.*\|.*logTest.* -delete
