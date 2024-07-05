INCLUDE_DIRS =
LIB_DIRS =
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS=

HFILES=
CFILES= segway_rt.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all: segway_rt 

clean:
	-rm -f *.o *.d
	-rm -f segway_rt  

segway_rt : segway_rt .o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread -lrt

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

