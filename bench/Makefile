SOEM ?= $(HOME)/ethercat/SOEM
CC ?= cc
CFLAGS = -O2 -Wall -D_GNU_SOURCE \
  -I$(SOEM)/soem -I$(SOEM)/osal -I$(SOEM)/osal/linux -I$(SOEM)/oshw/linux -I$(SOEM)/oshw

libecrt.a: libecrt.o
	ar rcs $@ $^

libecrt.o: libecrt.c libecrt.h
	$(CC) $(CFLAGS) -c -o $@ libecrt.c

clean:
	rm -f libecrt.o libecrt.a
