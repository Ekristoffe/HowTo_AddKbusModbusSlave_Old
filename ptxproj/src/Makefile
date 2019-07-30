VERSION = 1.3.0

CFLAGS=-c -Wall -Wextra -Os
CFLAGS += -I$(SYSROOT)/usr/include/OsLinux/
CFLAGS += -I$(SYSROOT)/usr/include/dal/
CFLAGS += $(shell $(PKG_CONFIG) --cflags wago_diagnostic)
CFLAGS += -I$(SYSROOT)/usr/include/diagnostic/

CFLAGS += -DVERSION=\"$(VERSION)\"
#needed for CROSS_COMPILE
#CFLAGS+=-fprofile-arcs -ftest-coverage
#CFLAGS += --static
CC = $(CROSS_COMPILE)gcc

#LDFLAGS += -lgcov -coverage
SOURCES =  main.c
SOURCES += utils.c
SOURCES += kbus.c
SOURCES += modbus.c
SOURCES += proc.c
SOURCES += modbus_watchdog.c
SOURCES += modbus_config.c
SOURCES += modbus_mac.c
SOURCES += modbus_kbusInfo.c
SOURCES += modbus_const.c
SOURCES += modbus_reply.c
SOURCES += modbus_shortDescription.c
SOURCES += conffile_reader.c
SOURCES += oms_led.c

LDFLAGS = -lpthread -ldbuskbuscommon 
LDFLAGS += $(SYSROOT)/usr/lib/libdal.a
LDFLAGS += -L/usr/lib/ -l:libmodbus.so.750
LDFLAGS += -lrt -lffi -lglib-2.0 -llibloader
LDFLAGS += -loms
LDFLAGS += $(shell $(PKG_CONFIG) --libs wago_diagnostic)
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=kbusmodbusslave

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(OBJECTS)


zip: clean
	@mkdir -p $(EXECUTABLE)-$(VERSION)
	@cp ../kbusmodbusslave.conf $(EXECUTABLE)-$(VERSION)/
	@cp $(SOURCES) *.h Makefile $(EXECUTABLE)-$(VERSION)/
	@tar -cjvRf ../$(EXECUTABLE)-$(VERSION).tar.bz2 $(EXECUTABLE)-$(VERSION)/*
	@rm -rf $(EXECUTABLE)-$(VERSION)
	@echo "..:: Done ::.."
