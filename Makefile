.PHONY: all clean

NAME = MyApp

VPATH = src

CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

# Pylon dependencies (Basler camera)
PYLON_ROOT = /opt/pylon5
LDFLAGS    = $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     = $(shell $(PYLON_ROOT)/bin/pylon-config --libs)
CPPFLAGS   = -I include -I/opt/pylon5/include -std=c++11

# OpenCV dependencies
LDFLAGS   += $(shell pkg-config --libs opencv)

# LibUSB dependencies
LDFLAGS   += -lm -lusb-1.0
LDLIBS    += -L/usr/local/lib

$(NAME): $(OBJ_FILES)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

obj/%.o: src/%.cpp
	$(CXX) $(CPPFLAGS) -c -o $@ $<
	
clean:
	$(RM) $(NAME).o $(NAME) $(OBJ_FILES)
	
