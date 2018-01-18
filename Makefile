
CXXFLAGS=-std=c++11 -pedantic -Wall -O3 -g

#CXX=clang++-3.9 
#CXX=clang++-4.0
#CXX=clang++
CC=$(CXX)

LDFLAGS=-g

puzzle: puzzle.o

puzzle.o: puzzle.cpp astar.h

.PHONY: clean

clean:
	-rm puzzle.o puzzle

