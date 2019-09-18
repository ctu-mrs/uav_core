CXX=g++
FLAGS=$(shell pkg-config --cflags --libs opencv) -g

.PHONY: clean

all:test

test: test.cpp
	$(CXX) $(FLAGS) $^ -o $@

clean:
	rm -rf test
