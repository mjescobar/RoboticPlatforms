EXECUTABLE = Crabot
VPATH = ./src ./headers 
CC = g++ -O3
CFLAGS = -g -Wall -I./src -I./headers  -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread -L. -lhyperneat -lrobotlib
DATE = `date +'%F--%T'`
VAR = ./saveTrains/$(DATE)

all: Crabot.o CalcFunctions.o Fitness.o SimFiles.o
	@mkdir -p bin/functions_files
	@mkdir -p bin/error_files
	@mkdir -p bin/simulation_files
	@mkdir -p bin/simulation_files/robot_position
	@mkdir -p bin/simulation_files/joints_position
	@mkdir -p bin/NEAT_organisms
	@rm -f ./bin/test
	@rm -f ./bin/functions_files/*
	@rm -f ./bin/error_files/*
	@rm -f ./bin/simulation_files/robot_position/*
	@rm -f ./bin/simulation_files/joints_position/*
	@rm -f ./bin/simulation_files/*.txt
	@rm -f ./bin/NEAT_organisms/*
	@echo "Linking all objects to ./bin/Crabot"
	@$(CC) $(CFLAGS) ./objects/Fitness.o ./objects/SimFiles.o ./objects/CalcFunctions.o ./objects/Crabot.o  -o ./bin/$(EXECUTABLE) $(LDFLAGS)

Crabot.o: Crabot.cpp 
	@mkdir -p objects
	@echo "Compiling Crabot.cpp to Crabot.o"
	@$(CC) $(CFLAGS) -c ./src/Crabot.cpp -o ./objects/Crabot.o

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
	@rm -f -R ./objects
	@rm -f ./bin/$(EXECUTABLE)
	@rm -f ./bin/test
	@rm -f -R ./bin/functions_files
	@rm -f -R ./bin/error_files
	@rm -f -R ./bin/NEAT_organisms
	@rm -f -R ./bin/simulation_files

save:
	@mkdir -p $(VAR); cp -R ./bin/* $(VAR);

