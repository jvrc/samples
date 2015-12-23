MOTIONS = $(wildcard *.yaml)
CONFS = $(wildcard *.conf)
DESTDIR = /usr
INSTALL = cp -p

.PHONY: clean

all: $(TARGET)

install:
	mkdir -p $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(MOTIONS) $(DESTDIR)/share/choreonoid-1.5/motion
	$(INSTALL) $(CONFS) $(DESTDIR)/lib/choreonoid-1.5/rtc
