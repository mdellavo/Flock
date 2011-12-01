
CC=gcc
CFLAGS=-Wall -std=c99 -O3 
LDFLAGS=`sdl-config --cflags` `sdl-config --libs` -lGL -lGLU 

.PHONY: all clean

all: flock

flock: flock.c
	$(CC) $(CFLAGS) -o flock flock.c $(LDFLAGS)

clean:
	rm flock
