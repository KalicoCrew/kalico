CC ?= gcc
CFLAGS = -O2 -Wall -Werror -D_GNU_SOURCE -pthread -I..
LDFLAGS = -ldl -lpthread

all: test_shim

test_shim: test_shim.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: test_shim
	./test_shim

clean:
	rm -f test_shim
.PHONY: all run clean
