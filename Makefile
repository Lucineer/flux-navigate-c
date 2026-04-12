CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2

all: test

libnavigate.o: navigate.c navigate.h
	$(CC) $(CFLAGS) -c navigate.c -o libnavigate.o

test: test_navigate.c navigate.c navigate.h
	$(CC) $(CFLAGS) -o test test_navigate.c navigate.c -lm
	./test

clean:
	rm -f libnavigate.o test

.PHONY: all test clean
