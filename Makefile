# Variables
CXX = g++
CXXFLAGS = -g -Wall -m64 -shared -fPIC 
LIBS = -lpthread
TARGET = control_system.dll
SRC = control_system.cpp

# Default target
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f $(TARGET)

run: 
	python control_system.py
