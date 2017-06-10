CXX = g++
INCLUDES= -I./ -I.. -Imock -IUnitTest/mock
CXXFLAGS = -g $(INCLUDES) -std=c++11 

OBJ = Curve.o MatrixR.o DistanceSensor.o RobotConfig.o Robot.o Constants.o Controller.o Motor.o PIDController.o RobotSound.o
LINKFLAGS= -lCppUTest -lCppUTestExt

.PHONY: default unittest clean

default: unittest
	./unittest

unittest: RobotTest.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ RobotTest.cpp *.o $(LINKFLAGS) $(LINKFLAGSLOG4) $(LIBLOG)

mock: ./mock/Arduino.cpp
	$(CXX) $(CXXFLAGS) -c ./mock/Arduino.cpp

include $(OBJ:.o=.d)

%.o: ../%.cpp
	$(CXX) $(CXXFLAGS) -c $<

%.d: ../%.cpp
	$(CXX) -MM $(CXXFLAGS) ../$*.cpp > $*.d

clean:
	rm *.o *.d unittest