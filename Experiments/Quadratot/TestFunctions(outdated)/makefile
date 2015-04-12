EXECUTABLE = TestQuadratot
CC = g++ -O3
CFLAGS = -g -Wall -I../../VREP/remoteApi -I../../VREP/include -I../../VREP/robotSimulator/objects -I../../VREP/robotSimulator/headers -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

all: TestQuadratot.o
	@mkdir -p bin
	@echo "Linking all objects to ./bin/$(EXECUTABLE)"
	@cd ../../VREP; make
	@$(CC) $(CFLAGS) TestQuadratot.o ../../VREP/remoteApi/extApiPlatform.o ../../VREP/remoteApi/extApi.o ../../VREP/robotSimulator/objects/RobotSimulator.o ../../VREP/robotSimulator/objects/Object.o ../../VREP/robotSimulator/objects/Dummy.o ../../VREP/robotSimulator/objects/Joint.o -o ./bin/$(EXECUTABLE) $(LDFLAGS)

TestQuadratot.o: TestQuadratot.cpp 
	@echo "Compiling TestQuadratot.cpp to TestQuadratot.o"
	@$(CC) $(CFLAGS) -c TestQuadratot.cpp -o TestQuadratot.o

clean:
	@rm -f *.o
	@rm -f -R ./bin
	@rm -f *_FUNCTIONS.hpp

function:
	@cp ../Train/bin/functions_files/* .
	