
CC = g++
CFLAGS = -g -Wall


all: RobotCommand_Generator

RobotCommand_Generator: main.cpp Vec.cpp Matrix.cpp Robot_IK_Solver.cpp
	$(CC) main.cpp Vec.cpp Matrix.cpp Robot_IK_Solver.cpp $(CFLAGS) -o RobotCommand_Generator
	
clean:
	rm *.txt RobotCommand_Generator
