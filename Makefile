CXX = g++
CXXFLAGS = -O3 -Wall -Wextra

all : bin bin/construct bin/query bin/benchmark

bin :
	mkdir bin

bin/construct : sample/construct.cpp src/pruned_highway_labeling.h
	$(CXX) $(CXXFLAGS) -Isrc -o $@ $^

bin/query : sample/query.cpp src/pruned_highway_labeling.h
	$(CXX) $(CXXFLAGS) -Isrc -o $@ $^

bin/benchmark : sample/benchmark.cpp src/pruned_highway_labeling.h
	$(CXX) $(CXXFLAGS) -Isrc -o $@ $^

clean :
	rm -rf bin

.PHONY : all clean