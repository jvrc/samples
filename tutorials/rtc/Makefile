CC     = g++
LDFLAGS= `pkg-config choreonoid-body-plugin openrtm-aist --libs --cflags` -shared -fPIC -lboost_system -lboost_filesystem
CFLAGS = -Wall
LIBS   =
SRCS   = $(wildcard *.cpp)
MOTIONS = $(wildcard *.yaml)
CONFS = $(wildcard *.conf)
TARGET = $(SRCS:.cpp=.so)
DESTDIR = /usr/local
INSTALL = cp -p

.PHONY: clean

all: $(TARGET)

RobotControllerRTC.so : RobotControllerRTC.cpp
	$(CC) $(CFLAGS) -o $@ RobotControllerRTC.cpp $(LDFLAGS)

RobotTorqueControllerRTC.so : RobotTorqueControllerRTC.cpp
	$(CC) $(CFLAGS) -o $@ RobotTorqueControllerRTC.cpp $(LDFLAGS)

RobotSensorsControllerRTC.so : RobotSensorsControllerRTC.cpp
	$(CC) $(CFLAGS) -o $@ RobotSensorsControllerRTC.cpp $(LDFLAGS)

clean:
	rm -rf $(OBJS) $(TARGET)

install:
	mkdir -p $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(TARGET) $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(MOTIONS) $(DESTDIR)/share/choreonoid-1.5/motion
	$(INSTALL) $(CONFS) $(DESTDIR)/lib/choreonoid-1.5/rtc
