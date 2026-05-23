CC ?= gcc
CFLAGS := -Wall -Wextra -O2 -fPIC -shared -pthread
LDFLAGS := -ldl -lrt -pthread

all: libvtime.so libsim_intercept.so

libvtime.so: libvtime.c vtime.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

libsim_intercept.so: libsim_intercept.c libsim_intercept.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f libvtime.so libsim_intercept.so

.PHONY: all clean
