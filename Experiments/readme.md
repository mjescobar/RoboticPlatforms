This folder contain some test with two differents types of robots, Crabot and Quadratot.

For run any program of this folder you must follow the following steps:

-Compile and install the libraries used in the tests:

In HyperNeat folder:

```
$ make
$ sudo make install
```

In RobotLib folder:

```
$ make
$ sudo make install
```

-Compile the program to be executed  in the respective folder:
```
$ make
```

The executable will be created in .(PROGRAM FOLDER)/bin

Before running any program you must first open the simulator program with the correct scene.
The simulation program used is V-REP PRO EDU available in:

http://www.coppeliarobotics.com/downloads.html

Please be sure you download the appropriate version for your operating system.

Before opening the program also make sure that the port associated with it is correct:
Open the file "remoteApiConnections.txt" in the program folder and assign 19998 to portIndex1_port.


Programs:

-Quadratot: 

Train:	Training walk for a quadruped robot. To run the executable must be added as argument a json hyperneat configuration file, user definitions and genetic encoding.
```
$ ./Quadratot mo_quadratot3.json user_def genetic_encoding
```
Test_neatOrganism: Test final results of Quadratot. To run the executable must be added as argument a json hyperneat configuration file, user definitions and a final genetic encoding to test some Neat organism. 
```
$ ./Quadratot mo_quadratot3.json user_def NEATG5P5
```

-Crabot: 

Train:	Training walk for a hexapod robot. To run the executable must be added as argument a json hyperneat configuration file, user definitions and genetic encoding.
```
$ ./Crabot mo_quadratot3.json user_def genetic_encoding
```
Test_neatOrganism: Test final results of Crabot. To run the executable must be added as argument a json hyperneat configuration file, user definitions and a final genetic encoding to test some Neat organism. 
```
$ ./Crabot mo_quadratot3.json user_def NEATG5P5
```