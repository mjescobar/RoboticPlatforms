EXECUTABLE_1 = example_dynamixel
EXECUTABLE_2 = example_vrep
CC = g++ -O3
CFLAGS = -g -Wall 
LDFLAGS = -lpthread -L. -lrobotlib

all: example_dynamixel.o example_vrep.o 
	@mkdir -p bin
	@mkdir -p error_files
	@rm -f /error_files/*
	@echo "Linking all objects to ./bin/example_dynamixel"
	@$(CC) $(CFLAGS) example_dynamixel.o -o ./bin/$(EXECUTABLE_1) $(LDFLAGS)
	@echo "Linking all objects to ./bin/example_vrep"
	@$(CC) $(CFLAGS) example_vrep.o -o ./bin/$(EXECUTABLE_2) $(LDFLAGS)

example_dynamixel.o: example_dynamixel.cpp 
	@echo "Compiling example_dynamixel.cpp to example_dynamixel.o"
	@$(CC) $(CFLAGS) -c example_dynamixel.cpp -o example_dynamixel.o

example_vrep.o: example_vrep.cpp 
	@echo "Compiling example_vrep.cpp to example_vrep.o"
	@$(CC) $(CFLAGS) -c example_vrep.cpp -o example_vrep.o

dynamixel: 
	./bin/$(EXECUTABLE_1)

vrep:
	./bin/$(EXECUTABLE_2)

clean:
	@rm -f *.o
	@rm -f -R ./bin
	@rm -f -R ./error_files
