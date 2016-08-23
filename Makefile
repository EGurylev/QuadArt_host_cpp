.PHONY: all clean

NAME = MyApp

VPATH = src

# Pylon dependencies (Basler camera)
PYLON_ROOT = /opt/pylon5
LDFLAGS    = $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     = $(shell $(PYLON_ROOT)/bin/pylon-config --libs)
CPPFLAGS   = -I include -I/opt/pylon5/include -std=c++11

# Qt dependencies
LDFLAGS   += -lQt5Core -lQt5Gui -lQt5Widgets
LDLIBS    += -L/usr/lib/x86_64-linux-gnu
CPPFLAGS  += -g -I/usr/include/qt5/ -std=c++11 -fPIC -pipe

# OpenCV dependencies
LDFLAGS   += $(shell pkg-config --libs opencv)

# LibUSB dependencies
LDFLAGS   += -lm -lusb-1.0
LDLIBS    += -L/usr/local/lib

$(NAME): main.o control_loop.o camera.o img_proc.o pose_estim.o Crazyflie.o Crazyradio.o pid.o moc.o
	$(CXX) -o $@ $^ $(LDFLAGS)  $(LDLIBS)
Crazyflie.o: Crazyflie.cpp
	$(CXX) $(CPPFLAGS) -c $<
Crazyradio.o: Crazyradio.cpp
	$(CXX) $(CPPFLAGS) -c $<
pid.o: pid.cpp
	$(CXX) $(CPPFLAGS) -c $<
pose_estim.o: pose_estim.cpp
	$(CXX) $(CPPFLAGS) -c $<
control_loop.o: control_loop.cpp
	$(CXX) $(CPPFLAGS)  -c $<
img_proc.o: img_proc.cpp
	$(CXX) $(CPPFLAGS) -c $<
camera.o: camera.cpp
	$(CXX) $(CPPFLAGS) -DUSE_GIGE  -c $<
main.o: main.cpp
	$(CXX) $(CPPFLAGS) -c $<
#moc is needed for working signals and slots mechanism in Qt 
moc.o: moc.cpp
	$(CXX)	$(CPPFLAGS) -c $<
moc.cpp: include/control_loop.h include/camera.h include/img_proc.h include/pose_estim.h include/common.h include/Crazyflie.h include/Crazyradio.h include/crtp.h include/pid.h
	moc $< -o $@
	
clean:
	$(RM) $(NAME).o $(NAME) control_loop.o camera.o main.o moc.o img_proc.o pose_estim.o moc.cpp Crazyflie.o Crazyradio.o pid.o
	
