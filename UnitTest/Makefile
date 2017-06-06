CXX = g++
INCLUDES= -I./
CXXFLAGS = -g $(INCLUDES) -std=c++11
SRC= ../Curve.cpp ../MatrixR.cpp ../DistanceSensor.cpp ../RobotConfig.cpp
OBJ = Curve.o MatrixR.o DistanceSensor.o RobotConfig.o
LINKFLAGS= -lCppUTest -lCppUTestExt

.PHONY: default unittest clean

default: unittest
	./unittest

unittest: RobotTest.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ RobotTest.cpp *.o $(LINKFLAGS) $(LINKFLAGSLOG4) $(LIBLOG)


include $(OBJ:.o=.d)

%.o: ../%.cpp
	$(CXX) $(CXXFLAGS) -c $<

%.d: ../%.cpp
	$(CXX) -MM $(CFLAGS)../$*.cpp > $*.d

clean:
	rm *.o *.d unittest