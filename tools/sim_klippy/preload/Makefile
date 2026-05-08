CC ?= gcc
CFLAGS = -O2 -Wall -Werror -fPIC -D_GNU_SOURCE -pthread
LDFLAGS = -shared -ldl -lpthread

all: libsim_intercept.so

libsim_intercept.so: libsim_intercept.c libsim_intercept.h
	$(CC) $(CFLAGS) -o $@ libsim_intercept.c $(LDFLAGS)

clean:
	rm -f libsim_intercept.so

.PHONY: all clean
