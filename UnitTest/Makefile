CXX = g++
INCLUDES= -I./
CXXFLAGS = -g $(INCLUDES) -std=c++11
SRCM= ../Curve.cpp ../MatrixR.cpp
OBJM = $(SRCM:.cpp=.o)
LINKFLAGS= -lCppUTest -lCppUTestExt

test: unittest
	./unittest

unittest: RobotTest.cpp $(OBJM)
	$(CXX) $(CXXFLAGS) -o $@ RobotTest.cpp *.o $(LINKFLAGS) $(LINKFLAGSLOG4) $(LIBLOG)

# Default compile

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $<
	
clean:
	rm *.o