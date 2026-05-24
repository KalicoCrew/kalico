CC ?= gcc
CFLAGS := -Wall -Wextra -O2 -fPIC -shared -pthread
LDFLAGS := -ldl -lrt -pthread

all: libvtime.so libvtime_clock_only.so libsim_intercept.so

libvtime.so: libvtime.c vtime.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

libvtime_clock_only.so: libvtime_clock_only.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

libsim_intercept.so: libsim_intercept.c libsim_intercept.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f libvtime.so libvtime_clock_only.so libsim_intercept.so

.PHONY: all clean
