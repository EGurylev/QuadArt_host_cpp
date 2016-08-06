.PHONY: all clean

NAME = MyApp

# Pylon dependencies (Basler camera)
PYLON_ROOT = /opt/pylon5
LDFLAGS    = $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     = $(shell $(PYLON_ROOT)/bin/pylon-config --libs)
CPPFLAGS   = -I/opt/pylon5/include -std=c++11

# Qt dependencies
LDFLAGS   += -lQt5Core -lQt5Gui -lQt5Widgets
LDLIBS    += -L/usr/lib/x86_64-linux-gnu
CPPFLAGS  += -I/usr/include/qt5/ -std=c++11 -fPIC -pipe

$(NAME): main.o control_loop.o moc.o
	$(CXX) -o $@ $^ $(LDFLAGS)  $(LDLIBS)
control_loop.o: control_loop.cpp control_loop.h
	$(CXX) $(CPPFLAGS) -DUSE_GIGE  -c $<
main.o: main.cpp
	$(CXX) $(CPPFLAGS) -c $<
#moc is needed for working signals and slots mechanism in Qt 
moc.o: moc.cpp
	$(CXX)	$(CPPFLAGS) -c $<
moc.cpp: control_loop.h
	moc $< -o $@
	
clean:
	$(RM) $(NAME).o $(NAME) control_loop.o main.o moc.o moc.cpp
	
