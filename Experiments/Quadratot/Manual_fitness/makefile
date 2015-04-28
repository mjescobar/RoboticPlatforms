EXECUTABLE = Quadratot
VPATH = ./src ./headers 
CC = g++ -O3 -std=c++11
CFLAGS = -g -Wall -I./src -I./headers -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread -L. -lhyperneat -lrobotlib -lboost_system -lboost_filesystem

all: Quadratot.o CalcFunctions.o Fitness.o SimFiles.o
	@mkdir -p saveTrains
	@echo "Linking all objects to ./bin/Quadratot"
	@$(CC) $(CFLAGS) ./objects/Fitness.o ./objects/SimFiles.o ./objects/CalcFunctions.o ./objects/Quadratot.o -o ./bin/$(EXECUTABLE) $(LDFLAGS)

Quadratot.o: Quadratot.cpp 
	@mkdir -p objects
	@echo "Compiling Quadratot.cpp to Quadratot.o"
	@$(CC) $(CFLAGS) -c ./src/Quadratot.cpp -o ./objects/Quadratot.o

CalcFunctions.o: CalcFunctions.cpp 
	@mkdir -p objects
	@echo "Compiling CalcFunctions.cpp to CalcFunctions.o"
	@$(CC) $(CFLAGS) -c ./src/CalcFunctions.cpp -o ./objects/CalcFunctions.o

Fitness.o: Fitness.cpp
	@mkdir -p objects
	@echo "Compiling Fitness.cpp to Fitness.o"
	@$(CC) $(CFLAGS) -c ./src/Fitness.cpp -o ./objects/Fitness.o

SimFiles.o: SimFiles.cpp
	@mkdir -p objects
	@echo "Compiling SimFiles.cpp to SimFiles.o"
	@$(CC) $(CFLAGS) -c ./src/SimFiles.cpp -o ./objects/SimFiles.o

clean:
	@rm -f ./bin/$(EXECUTABLE)
	@rm -f -R ./objects


