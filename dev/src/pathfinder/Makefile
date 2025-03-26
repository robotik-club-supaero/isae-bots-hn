# FIXME!!! Convert to an Ament package.
# I'm getting an empty .so when I use CMake.

# Directories
PROJECT = pathfinder
SRCDIR = src
INCDIR = include
BUILDDIR = build

# Compiler and flags
CXX = c++
CXXFLAGS = -Wall -std=c++11 -I./$(INCDIR)
PYBIND11_FLAGS = -O3 -shared -fPIC $(shell python3 -m pybind11 --includes)

# Files
SOURCES = $(SRCDIR)/Map.cpp $(SRCDIR)/ObstacleCircle.cpp $(SRCDIR)/ObstacleRect.cpp $(SRCDIR)/Point.cpp $(SRCDIR)/Segment.cpp
EXEC = $(PROJECT)$(shell python3-config --extension-suffix)

# Targets
main:
	$(CXX) $(CXXFLAGS) $(PYBIND11_FLAGS) $(SRCDIR)/main.cpp $(SOURCES) -o $(BUILDDIR)/$(EXEC)

test:
	$(CXX) $(CXXFLAGS) $(SRCDIR)/test.cpp $(SOURCES) -o $(BUILDDIR)/test

# Clean build files
clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean
